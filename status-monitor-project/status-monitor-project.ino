#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>

//
// ------- Réglages matériels -------
//
#define ADC_PIN           2      // <-- change au besoin (GPIO ADC sur l'ESP32-C3 Mini)
#define SAMPLE_PERIOD_MS  5      // période d'échantillonnage
#define WINDOW_MS         3000   // fenêtre d’analyse pour la détection d’état
#define EMA_ALPHA         0.2f   // lissage (0..1) : plus grand = plus réactif

// Plage de fréquence considérée comme "clignotement rapide"
#define BLINK_MIN_HZ      1.5f
#define BLINK_MAX_HZ      10.0f

//
// ------- Réglages Wi-Fi AP -------
//
const char* AP_SSID = "TOne-Status-Monitor";
const char* AP_PASS = "12341234";

WebServer server(80);

//
// ------- Variables de mesure -------
//
uint16_t adcRaw = 0;
float    ema = 0.0f;

uint16_t rollingMin = 4095;
uint16_t rollingMax = 0;

bool     levelHigh = false;      // état binaire après seuil + hystérésis
bool     lastLevelHigh = false;

const uint16_t HYST = 50;        // hystérésis en points ADC (~1% de 12 bits)

struct Edge {
  unsigned long t;
};
const size_t MAX_EDGES = 128;
Edge edges[MAX_EDGES];
size_t edgeCount = 0;

unsigned long lastSample = 0;
unsigned long lastWindowPurge = 0;

enum SystemState { STATE_OFF=0, STATE_SOLID=1, STATE_BLINK=2 };
SystemState currentState = STATE_OFF;
SystemState lastReportedState = STATE_OFF;

unsigned long stateSinceMs = 0;

//
// ------- HTML/CSS + JS -------
//
const char* PAGE_INDEX = R"HTML(
<!doctype html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PAC Sensor</title>
<style>
  :root { color-scheme: dark; }
  body {
    margin: 0; font-family: system-ui, -apple-system, Segoe UI, Roboto, Ubuntu, Cantarell, "Helvetica Neue", Arial, "Noto Sans", sans-serif;
    background: #0b0f14; color: #e6e9ef;
    display: grid; place-items: center; min-height: 100vh;
  }
  .card {
    background: #0f1621; border: 1px solid #1f2a38; border-radius: 16px; padding: 24px 28px; width: min(680px, 92vw);
    box-shadow: 0 10px 30px rgba(0,0,0,.35);
  }
  h1 { margin: 0 0 8px; font-size: 26px; letter-spacing: .3px; color: #c8d3e0; }
  .muted { color: #8a97a8; font-size: 14px; }
  .status {
    margin-top: 18px; display: flex; align-items: center; gap: 12px;
    padding: 14px 16px; border-radius: 12px; border: 1px solid #223045;
    background: linear-gradient(180deg, #101725, #0e1420);
  }
  .dot {
    width: 14px; height: 14px; border-radius: 8px; display: inline-block;
    box-shadow: 0 0 18px currentColor;
  }
  .state { font-size: 18px; font-weight: 600; letter-spacing: .2px; }
  .since { font-size: 14px; color: #a2afc2; }
  .grid { display:grid; grid-template-columns: 1fr 1fr; gap: 16px; margin-top: 16px; }
  .tile {
    background:#0c121c; border:1px solid #1f2a38; border-radius:12px; padding:12px 14px;
  }
  .k { font-size:12px; color:#8a97a8; }
  .v { font-size:16px; margin-top:4px; }
  @media (max-width:600px){ .grid{ grid-template-columns:1fr; } }
</style>
</head>
<body>
  <div class="card">
    <h1>État du chauffage</h1>
    <div class="muted">Par LDR collée sur la LED</div>
    <div class="status">
      <span id="dot" class="dot" style="color:#888"></span>
      <div>
        <div id="state" class="state">—</div>
        <div id="since" class="since">—</div>
      </div>
    </div>
    <div class="grid">
      <div class="tile"><div class="k">Fréquence estimée</div><div id="freq" class="v">—</div></div>
      <div class="tile"><div class="k">Niveau LDR (0–4095)</div><div id="lvl" class="v">—</div></div>
    </div>
    <div class="muted" style="margin-top:14px">Le statut se met à jour automatiquement.</div>
  </div>
<script>
  function humanize(ms){
    const s=Math.floor(ms/1000), d=Math.floor(s/86400), h=Math.floor((s%86400)/3600),
          m=Math.floor((s%3600)/60), sec=s%60;
    const parts=[];
    if(d) parts.push(d+" j"); if(h) parts.push(h+" h"); if(m) parts.push(m+" min"); parts.push(sec+" s");
    return parts.join(" ");
  }
  function colorFor(state){
    if(state==="ON (fixe)") return "#2dcc70";
    if(state==="CLIGNOTE (panne)") return "#f1c40f";
    if(state==="OFF") return "#e74c3c";
    return "#888";
  }
  async function refresh(){
    try{
      const r = await fetch("/status");
      const j = await r.json();
      const map = { "SOLID":"ON (fixe)", "BLINK":"CLIGNOTE (panne)", "OFF":"OFF" };
      const stateTxt = map[j.state] || "—";
      document.getElementById("state").textContent = stateTxt;
      document.getElementById("since").textContent = "Depuis " + humanize(j.since_ms);
      document.getElementById("freq").textContent = j.freq_hz.toFixed(2) + " Hz";
      document.getElementById("lvl").textContent  = j.level;
      const c = colorFor(stateTxt);
      const dot = document.getElementById("dot");
      dot.style.color = c;
      dot.style.boxShadow = "0 0 18px " + c;
    }catch(e){ /* ignore */ }
  }
  setInterval(refresh, 1000);
  refresh();
</script>
</body>
</html>
)HTML";

//
// ------- Helpers -------
//
void addEdge(unsigned long t) {
  if (edgeCount < MAX_EDGES) {
    edges[edgeCount++].t = t;
  } else {
    // décalage si saturé
    memmove(&edges[0], &edges[1], (MAX_EDGES-1)*sizeof(Edge));
    edges[MAX_EDGES-1].t = t;
  }
}

void purgeOldEdges(unsigned long now) {
  // garde seulement les fronts des WINDOW_MS dernières ms
  size_t w = 0;
  for (size_t i = 0; i < edgeCount; ++i) {
    if (now - edges[i].t <= WINDOW_MS) {
      edges[w++] = edges[i];
    }
  }
  edgeCount = w;
}

float estimateFrequencyHz() {
  if (edgeCount < 2) return 0.0f;
  // On compte les intervalles entre fronts alternés (approx.)
  unsigned long span = edges[edgeCount-1].t - edges[0].t;
  if (span == 0) return 0.0f;
  // Chaque cycle a ~2 fronts -> cycles = (edgeCount-1)/2
  float cycles = (edgeCount - 1) / 2.0f;
  return (cycles * 1000.0f) / (float)span;
}

const char* stateToText(SystemState s) {
  switch (s) {
    case STATE_SOLID: return "SOLID";
    case STATE_BLINK: return "BLINK";
    default: return "OFF";
  }
}

SystemState decideState(unsigned long now, float freqHz, float highRatio, size_t edgesInWin) {
  // Heuristique robuste
  if (edgesInWin >= 4 && freqHz >= BLINK_MIN_HZ && freqHz <= BLINK_MAX_HZ) {
    return STATE_BLINK;
  }
  if (edgesInWin <= 1) {
    if (highRatio > 0.8f) return STATE_SOLID;
    if (highRatio < 0.2f) return STATE_OFF;
  }
  // sinon: garder l'état précédent pour éviter le "flap"
  return currentState;
}

//
// ------- HTTP Handlers -------
//
void handleRoot() {
  server.send(200, "text/html; charset=utf-8", PAGE_INDEX);
}

void handleStatus() {
  float freq = estimateFrequencyHz();

  // calcul du ratio "clair" sur la fenêtre actuelle à partir des edges
  // approximation: si le niveau actuel est "high", on considère ~50% si pas de fronts (stable)
  // pour rester simple, on expose au moins le niveau actuel.
  // (Option: pourrait accumuler un compteur highTime, mais suffisant ici)
  static unsigned long lastReport = 0;
  static float reportedFreq = 0.0f;
  if (millis() - lastReport > 200) { reportedFreq = freq; lastReport = millis(); }

  String json = "{";
  json += "\"state\":\""; json += stateToText(currentState); json += "\",";
  json += "\"since_ms\":"; json += String(millis() - stateSinceMs); json += ",";
  json += "\"level\":";    json += String((int)ema); json += ",";
  json += "\"freq_hz\":";  json += String(reportedFreq, 3);
  json += "}";
  server.send(200, "application/json; charset=utf-8", json);
}

//
// ------- Setup / Loop -------
//
void setup() {
  Serial.begin(115200);
  delay(200);

  // ADC config
  analogReadResolution(12);     // 0..4095
  analogSetAttenuation(ADC_11db); // bonne plage 0..~3.3V

  // Init EMA avec première lecture
  adcRaw = analogRead(ADC_PIN);
  ema = adcRaw;
  rollingMin = adcRaw;
  rollingMax = adcRaw;
  levelHigh = lastLevelHigh = false;
  stateSinceMs = millis();

  // Démarre l'AP
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  if (ok) {
    Serial.print("AP démarré: "); Serial.println(AP_SSID);
    Serial.print("IP: "); Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Échec démarrage AP");
  }

  // Web server
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.begin();
}

void loop() {
  server.handleClient();

  const unsigned long now = millis();
  if (now - lastSample >= SAMPLE_PERIOD_MS) {
    lastSample = now;

    // Lecture + lissage
    adcRaw = analogRead(ADC_PIN);
    ema = (EMA_ALPHA * adcRaw) + (1.0f - EMA_ALPHA) * ema;

    // Mise à jour min/max glissants pour seuil auto (dérive lente)
    rollingMin = min<uint16_t>(rollingMin + 1, (uint16_t)ema); // dérive vers le haut lentement
    rollingMax = max<uint16_t>(rollingMax - 1, (uint16_t)ema); // dérive vers le bas lentement
    if (ema < rollingMin) rollingMin = ema;
    if (ema > rollingMax) rollingMax = ema;

    // Seuil + hystérésis
    uint16_t mid = (rollingMin + rollingMax) / 2;
    uint16_t thHi = mid + HYST;
    uint16_t thLo = mid - HYST;

    if (!levelHigh && ema > thHi) levelHigh = true;
    else if (levelHigh && ema < thLo) levelHigh = false;

    // Détection de fronts
    if (levelHigh != lastLevelHigh) {
      addEdge(now);
      lastLevelHigh = levelHigh;
    }

    // Purge des fronts hors fenêtre
    if (now - lastWindowPurge >= 250) {
      lastWindowPurge = now;
      purgeOldEdges(now);

      // Estimation fréquence (sur edges conservés)
      float freq = estimateFrequencyHz();

      // Estime ratio "clair" grossièrement via position EMA dans [min,max]
      float span = max<int>(1, (int)rollingMax - (int)rollingMin);
      float highRatio = constrain((ema - rollingMin) / span, 0.0f, 1.0f);

      SystemState newState = decideState(now, freq, highRatio, edgeCount);

      if (newState != currentState) {
        currentState = newState;
        stateSinceMs = now;
        Serial.print("Nouvel état: "); Serial.print(stateToText(currentState));
        Serial.print(" | freq≈"); Serial.print(freq, 2);
        Serial.print(" Hz | level="); Serial.println(ema);
      }
    }
  }
}
