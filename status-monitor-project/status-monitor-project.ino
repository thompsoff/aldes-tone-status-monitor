#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_now.h>
#include <esp_wifi.h>

//
// ------- Réglages matériels -------
//
#define ADC_PIN             2      // GPIO ADC connecté au pont LDR
#define SAMPLE_PERIOD_MS    5
#define EMA_ALPHA           0.2f   // lissage lecture
#define ADC_MAX             4095.0f

#define LED_PIN             8
#define LED_ACTIVE_LOW      true   // <-- passe à false si LED active niveau haut

// Fenêtre d'analyse pour reconnaitre un clignotement
#define WINDOW_MS           3000
#define BLINK_MIN_HZ        1.5f
#define BLINK_MAX_HZ        10.0f

//
// ------- Wi-Fi AP -------
//
const char* AP_SSID = "TOne-Status-Monitor";
const char* AP_PASS = "12341234";
WebServer server(80);

//
// ------- Persistance seuils -------
//
Preferences prefs;
float threshOnPct  = 60.0f; // défaut
float threshOffPct = 40.0f; // défaut

//
// ------- Mesures/état -------
//
uint16_t adcRaw = 0;
float    ema = 0.0f;

bool ledIsOn = false;       // état binaire issu des seuils
bool lastLedIsOn = false;

enum SystemState { STATE_OFF=0, STATE_SOLID=1, STATE_BLINK=2 };
SystemState currentState = STATE_OFF;
unsigned long stateSinceMs = 0;

struct Edge { unsigned long t; };
const size_t MAX_EDGES = 128;
Edge edges[MAX_EDGES];
size_t edgeCount = 0;
unsigned long lastSample = 0, lastWindowPurge = 0;

//
// ------- ESP-NOW (broadcast reboot quand BLINK >= 10s) -------
//
uint8_t broadcastAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
constexpr unsigned long REBOOT_SEND_INTERVAL_MS = 1000;  // toutes les 1 s
constexpr unsigned long BLINK_HOLD_MS           = 10000; // BLINK depuis >= 10 s
unsigned long lastRebootSendMs = 0;
unsigned long blinkSinceMs = 0;
bool wasBlinking = false;

void onEspNowSent(const wifi_tx_info_t* /*tx_info*/, esp_now_send_status_t status) {
  Serial.print("ESP-NOW send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

//
// ------- UI (HTML/CSS/JS) -------
//
const char* PAGE_INDEX = R"HTML(
<!doctype html>
<html lang="fr">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>PAC Sensor</title>
<style>
  :root { color-scheme: dark; }
  body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,Cantarell,Helvetica Neue,Arial,Noto Sans,sans-serif;background:#0b0f14;color:#e6e9ef;display:grid;place-items:center;min-height:100vh}
  .wrap{width:min(820px,94vw);display:grid;gap:16px}
  .card{background:#0f1621;border:1px solid #1f2a38;border-radius:16px;padding:20px 22px;box-shadow:0 10px 30px rgba(0,0,0,.35)}
  h1{margin:0 0 6px;font-size:24px;color:#c8d3e0}
  .muted{color:#8a97a8;font-size:13px}
  .status{margin-top:12px;display:flex;align-items:center;gap:12px;padding:12px 14px;border-radius:12px;border:1px solid #223045;background:linear-gradient(180deg,#101725,#0e1420)}
  .dot{width:14px;height:14px;border-radius:8px;display:inline-block;box-shadow:0 0 18px currentColor}
  .state{font-size:18px;font-weight:600}
  .since{font-size:13px;color:#a2afc2}
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:12px}
  .tile{background:#0c121c;border:1px solid #1f2a38;border-radius:12px;padding:12px 14px}
  .k{font-size:12px;color:#8a97a8}.v{font-size:16px;margin-top:4px}
  @media(max-width:700px){.grid{grid-template-columns:1fr}}
  .form-row{display:grid;grid-template-columns:120px 1fr 60px;gap:10px;align-items:center;margin-top:10px}
  input[type=range]{width:100%}
  button{background:#1a2332;border:1px solid #2a3b55;border-radius:10px;color:#e6e9ef;padding:10px 14px;font-weight:600;cursor:pointer}
  button:active{transform:translateY(1px)}
</style>
</head>
<body>
  <div class="wrap">
    <div class="card">
      <h1>État du chauffage</h1>
      <div class="muted">Détection par LDR + seuils configurables</div>
      <div class="status">
        <span id="dot" class="dot" style="color:#888"></span>
        <div>
          <div id="state" class="state">—</div>
          <div id="since" class="since">—</div>
        </div>
      </div>
      <div class="grid">
        <div class="tile"><div class="k">Fréquence estimée</div><div id="freq" class="v">—</div></div>
        <div class="tile"><div class="k">Niveau LDR</div><div id="lvl" class="v">—</div></div>
        <div class="tile"><div class="k">LED (seuils)</div><div id="bin" class="v">—</div></div>
        <div class="tile"><div class="k">Seuils (ON / OFF)</div><div id="ths" class="v">—</div></div>
      </div>
    </div>

    <div class="card">
      <h2 style="margin:0 0 8px;font-size:18px;color:#c8d3e0">Réglages des seuils</h2>
      <div class="muted">ON = passe à ON si niveau ≥ seuil ON. OFF = repasse à OFF si niveau ≤ seuil OFF.</div>
      <div class="form-row">
        <label>Seuil ON</label>
        <input id="on" type="range" min="0" max="100" step="1">
        <div id="onv" style="text-align:right">—%</div>
      </div>
      <div class="form-row">
        <label>Seuil OFF</label>
        <input id="off" type="range" min="0" max="100" step="1">
        <div id="offv" style="text-align:right">—%</div>
      </div>
      <div style="display:flex;gap:10px;margin-top:12px">
        <button id="save">Enregistrer</button>
      </div>
      <div class="muted" id="msg" style="margin-top:10px"></div>
    </div>
  </div>

<script>
  function humanize(ms){
    const s=Math.floor(ms/1000), d=Math.floor(s/86400), h=Math.floor((s%86400)/3600),
          m=Math.floor((s%3600)/60), sec=s%60;
    const parts=[]; if(d) parts.push(d+" j"); if(h) parts.push(h+" h"); if(m) parts.push(m+" min"); parts.push(sec+" s");
    return parts.join(" ");
  }
  function colorFor(state){
    if(state==="ON (fixe)") return "#2dcc70";
    if(state==="CLIGNOTE") return "#f1c40f";
    if(state==="OFF") return "#e74c3c";
    return "#888";
  }
  async function loadConf(){
    const r = await fetch("/config"); const j = await r.json();
    const on = Math.round(j.th_on_pct), off = Math.round(j.th_off_pct);
    onEl.value = on; offEl.value = off;
    onv.textContent = on+"%"; offv.textContent = off+"%";
  }
  async function refresh(){
    const r = await fetch("/status"); const j = await r.json();
    const map = { "SOLID":"ON (fixe)", "BLINK":"CLIGNOTE", "OFF":"OFF" };
    const stateTxt = map[j.state] || "—";
    state.textContent = stateTxt;
    since.textContent = "Depuis " + humanize(j.since_ms);
    freq.textContent  = j.freq_hz.toFixed(2) + " Hz";
    lvl.textContent   = j.level_raw + " (" + j.level_pct.toFixed(1) + "%)";
    bin.textContent   = j.led_on ? "ON" : "OFF";
    ths.textContent   = j.th_on_pct.toFixed(0)+"% / "+j.th_off_pct.toFixed(0)+"%";
    const c = colorFor(stateTxt); dot.style.color=c; dot.style.boxShadow="0 0 18px "+c;
  }
  const onEl  = document.getElementById("on");
  const offEl = document.getElementById("off");
  const onv   = document.getElementById("onv");
  const offv  = document.getElementById("offv");
  const state = document.getElementById("state");
  const since = document.getElementById("since");
  const freq  = document.getElementById("freq");
  const lvl   = document.getElementById("lvl");
  const bin   = document.getElementById("bin");
  const ths   = document.getElementById("ths");
  const dot   = document.getElementById("dot");
  const msg   = document.getElementById("msg");

  onEl.addEventListener("input", e=>onv.textContent=e.target.value+"%");
  offEl.addEventListener("input", e=>offv.textContent=e.target.value+"%");
  document.getElementById("save").addEventListener("click", async ()=>{
    const on = parseInt(onEl.value,10);
    const off = parseInt(offEl.value,10);
    if(off>on){ msg.textContent="Le seuil OFF doit être ≤ seuil ON."; return; }
    try{
      const r = await fetch("/config", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({th_on_pct:on, th_off_pct:off})});
      msg.textContent = r.ok ? "Enregistré ✔️ (persisté)" : "Erreur d'enregistrement";
    }catch(e){ msg.textContent="Erreur de communication"; }
  });

  setInterval(refresh, 1000);
  loadConf(); refresh();
</script>
</body></html>
)HTML";

//
// ------- Helpers -------
//
void addEdge(unsigned long t){
  if(edgeCount < MAX_EDGES){ edges[edgeCount++].t = t; }
  else { memmove(&edges[0], &edges[1], (MAX_EDGES-1)*sizeof(Edge)); edges[MAX_EDGES-1].t = t; }
}
void purgeOldEdges(unsigned long now){
  size_t w=0; for(size_t i=0;i<edgeCount;i++){ if(now - edges[i].t <= WINDOW_MS) edges[w++]=edges[i]; }
  edgeCount = w;
}
float estimateFrequencyHz(){
  if(edgeCount<2) return 0.0f;
  unsigned long span = edges[edgeCount-1].t - edges[0].t; if(!span) return 0.0f;
  float cycles = (edgeCount - 1) / 2.0f; // ~2 fronts par cycle
  return (cycles * 1000.0f) / (float)span;
}
SystemState classify(bool ledOn, float freqHz, size_t edgesInWin){
  if(edgesInWin >= 4 && freqHz >= BLINK_MIN_HZ && freqHz <= BLINK_MAX_HZ) return STATE_BLINK;
  if(ledOn && edgesInWin <= 1)  return STATE_SOLID;
  if(!ledOn && edgesInWin <= 1) return STATE_OFF;
  return currentState;
}
void applyBoardLED(bool on){
  bool level = LED_ACTIVE_LOW ? !on : on;
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, level);
}

//
// ------- HTTP -------
//
void handleRoot(){ server.send(200, "text/html; charset=utf-8", PAGE_INDEX); }

void handleStatus(){
  float levelPct = (ema / ADC_MAX) * 100.0f;
  float freq = estimateFrequencyHz();
  String json = "{";
  json += "\"state\":\""  + String(currentState==STATE_SOLID?"SOLID":currentState==STATE_BLINK?"BLINK":"OFF") + "\",";
  json += "\"since_ms\":" + String(millis()-stateSinceMs) + ",";
  json += "\"level_raw\":"+ String((int)ema) + ",";
  json += "\"level_pct\":"+ String(levelPct,1) + ",";
  json += "\"freq_hz\":"  + String(freq,3) + ",";
  json += "\"led_on\":"   + String(ledIsOn ? "true":"false") + ",";
  json += "\"th_on_pct\":"+ String(threshOnPct,1) + ",";
  json += "\"th_off_pct\":"+ String(threshOffPct,1);
  json += "}";
  server.send(200, "application/json; charset=utf-8", json);
}

void handleConfigGet(){
  String json = "{";
  json += "\"th_on_pct\":"+ String(threshOnPct,1) + ",";
  json += "\"th_off_pct\":"+ String(threshOffPct,1);
  json += "}";
  server.send(200, "application/json; charset=utf-8", json);
}

void handleConfigPost(){
  if(server.hasArg("plain")){
    String body = server.arg("plain");
    float on=-1, off=-1;
    int i=body.indexOf("th_on_pct"); if(i>=0){ int c=body.indexOf(':',i); if(c>=0) on = body.substring(c+1).toFloat(); }
    i=body.indexOf("th_off_pct"); if(i>=0){ int c=body.indexOf(':',i); if(c>=0) off = body.substring(c+1).toFloat(); }
    if(on>=0 && off>=0 && off<=on && on<=100.0f && off>=0.0f){
      threshOnPct = on; threshOffPct = off;
      prefs.putFloat("on_pct",  threshOnPct);
      prefs.putFloat("off_pct", threshOffPct);
      server.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
      return;
    }
  }
  server.send(400, "application/json; charset=utf-8", "{\"ok\":false}");
}

//
// ------- Setup / Loop -------
//
void setup(){
  Serial.begin(115200); delay(150);

  // ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // NVS
  prefs.begin("config", false);
  if(prefs.isKey("on_pct"))  threshOnPct  = prefs.getFloat("on_pct",  threshOnPct);
  if(prefs.isKey("off_pct")) threshOffPct = prefs.getFloat("off_pct", threshOffPct);

  // Init première lecture
  adcRaw = analogRead(ADC_PIN);
  ema = adcRaw;
  ledIsOn = false;
  lastLedIsOn = false;
  currentState = STATE_OFF;
  stateSinceMs = millis();
  applyBoardLED(ledIsOn);

  // Wi-Fi en AP+STA (héberge l'UI et active ESP-NOW)
  WiFi.mode(WIFI_AP_STA);
  if(WiFi.softAP(AP_SSID, AP_PASS)){
    Serial.print("AP "); Serial.print(AP_SSID); Serial.print(" IP="); Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Échec AP");
  }

  // --- ESP-NOW init ---
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while(true){ delay(1000); }
  }
  esp_now_register_send_cb(onEspNowSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, broadcastAddress, 6);
  peer.channel = 0;     // 0 = canal Wi-Fi courant
  peer.encrypt = false; // broadcast non chiffré
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add broadcast peer");
    while(true){ delay(1000); }
  }

  // HTTP routes
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/config", HTTP_GET, handleConfigGet);
  server.on("/config", HTTP_POST, handleConfigPost);
  server.begin();

  Serial.println("Prêt : UI sur AP + ESP-NOW en broadcast quand BLINK >= 10s");
}

void loop(){
  server.handleClient();

  const unsigned long now = millis();
  if(now - lastSample >= SAMPLE_PERIOD_MS){
    lastSample = now;

    // Lecture + lissage
    adcRaw = analogRead(ADC_PIN);
    ema = (EMA_ALPHA * adcRaw) + (1.0f - EMA_ALPHA) * ema;

    // Conversion en %
    float levelPct = (ema / ADC_MAX) * 100.0f;

    // Hystérésis par seuils configurables
    if(!ledIsOn && levelPct >= threshOnPct) ledIsOn = true;
    else if(ledIsOn && levelPct <= threshOffPct) ledIsOn = false;

    // LED embarquée reflète l'état binaire
    applyBoardLED(ledIsOn);

    // Détection de fronts pour évaluer la fréquence (clignotement)
    if(ledIsOn != lastLedIsOn){ addEdge(now); lastLedIsOn = ledIsOn; }
  }

  // Purge + classification périodique
  if(now - lastWindowPurge >= 250){
    lastWindowPurge = now;
    purgeOldEdges(now);
    float freq = estimateFrequencyHz();
    SystemState newState = classify(ledIsOn, freq, edgeCount);
    if(newState != currentState){
      currentState = newState;
      stateSinceMs = now;
      Serial.print("État="); Serial.print(currentState==STATE_SOLID?"SOLID":currentState==STATE_BLINK?"BLINK":"OFF");
      Serial.print(" | led="); Serial.print(ledIsOn?"ON":"OFF");
      Serial.print(" | freq≈"); Serial.println(freq, 2);
    }
  }

  // --- Gestion BLINK >= 10s -> broadcast "reboot\0" toutes les 1 s ---
  bool blinkingNow = (currentState == STATE_BLINK);

  if (blinkingNow && !wasBlinking) {
    // début du clignotement
    blinkSinceMs = now;
    Serial.println("Blink started");
  } else if (!blinkingNow && wasBlinking) {
    // fin du clignotement
    Serial.println("Blink stopped");
  }
  wasBlinking = blinkingNow;

  const bool blinkLongEnough = blinkingNow && (now - blinkSinceMs >= BLINK_HOLD_MS);
  if (blinkLongEnough && (now - lastRebootSendMs >= REBOOT_SEND_INTERVAL_MS)) {
    lastRebootSendMs = now;

    // Buffer "reboot" + octet nul ajouté manuellement (7 octets)
    uint8_t msg[7] = { 'r','e','b','o','o','t','\0' };

    esp_err_t res = esp_now_send(broadcastAddress, msg, sizeof(msg)); // inclut bien le '\0'
    if (res != ESP_OK) {
      Serial.print("esp_now_send() error: ");
      Serial.println(res);
    } else {
      Serial.println("Sent: reboot\\0 (broadcast)");
    }
  }

  // petite respiration
  delay(2);
}
