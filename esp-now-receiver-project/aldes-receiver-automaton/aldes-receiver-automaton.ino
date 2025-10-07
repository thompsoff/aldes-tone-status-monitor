// === COMMON BLOCK ===
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Servo.h>

// ==== Version-agnostic callback argument macros ====
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
// Arduino core 3.x / ESP-IDF 5.x
  #define SEND_CB_ARGS  const wifi_tx_info_t* /*tx_info*/, esp_now_send_status_t status
  #define RECV_CB_ARGS  const esp_now_recv_info_t* /*info*/, const uint8_t* data, int len
#else
// Arduino core 2.x / ESP-IDF 4.x
  #define SEND_CB_ARGS  const uint8_t* /*mac*/, esp_now_send_status_t status
  #define RECV_CB_ARGS  const uint8_t* /*mac*/, const uint8_t* data, int len
#endif

// ====== LED (active-low friendly for ESP32-C3) ======
#ifndef LED_PIN
#define LED_PIN 8                 // built-in LED on many ESP32-C3 boards
#endif
#ifndef LED_ACTIVE_LOW
#define LED_ACTIVE_LOW 1          // C3 onboard LEDs are usually active-low (LOW=ON)
#endif

static inline void ledInit() {
  pinMode(LED_PIN, OUTPUT);
  // ensure LED is OFF at boot
  digitalWrite(LED_PIN, LED_ACTIVE_LOW ? HIGH : LOW);
}
static inline void ledWrite(bool on) {
  digitalWrite(LED_PIN, LED_ACTIVE_LOW ? (on ? LOW : HIGH) : (on ? HIGH : LOW));
}
static inline void blink(uint16_t ms = 50) {
  ledWrite(true);
  delay(ms);
  ledWrite(false);
}

// ====== ESP-NOW broadcast helper ======
uint8_t kBroadcastAddr[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static bool addBroadcastPeer() {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, kBroadcastAddr, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_is_peer_exist(kBroadcastAddr)) return true;
  return esp_now_add_peer(&peer) == ESP_OK;
}

// ====== Package definition (fits in 1 ESP-NOW frame) ======
#define MAX_DATA_LEN 232  // 250 - 16(UUID) - 2(length)
typedef struct __attribute__((packed)) {
  uint32_t uuid[4];          // 128-bit UUID (random)
  uint16_t length;           // bytes used in data[]
  char     data[MAX_DATA_LEN];
} struct_package;

// ====== UUID helpers ======
static inline bool uuid_equal(const uint32_t a[4], const uint32_t b[4]) {
  return (a[0]==b[0]) && (a[1]==b[1]) && (a[2]==b[2]) && (a[3]==b[3]);
}

// ====== Duplicate suppression: FIFO of 64 UUIDs ======
#define UUID_CACHE_CAP 64
struct UuidCache {
  uint32_t ids[UUID_CACHE_CAP][4];
  uint16_t count = 0;
  uint16_t head  = 0; // next insert index
} cache;

static inline bool cache_contains(const uint32_t uuid[4]) {
  for (uint16_t i = 0; i < cache.count; ++i) {
    uint16_t idx = (cache.head + UUID_CACHE_CAP - 1 - i) % UUID_CACHE_CAP;
    if (uuid_equal(cache.ids[idx], uuid)) return true;
  }
  return false;
}
static inline void cache_insert(const uint32_t uuid[4]) {
  memcpy(cache.ids[cache.head], uuid, sizeof(uint32_t) * 4);
  cache.head = (cache.head + 1) % UUID_CACHE_CAP;
  if (cache.count < UUID_CACHE_CAP) cache.count++;
}

// ====== Receiver-specific I/O ======
#ifndef SIGNAL_PIN
#define SIGNAL_PIN 4
#endif

static inline void signalInit() {
  pinMode(SIGNAL_PIN, OUTPUT);
  digitalWrite(SIGNAL_PIN, LOW);
}

// Blink LED and assert SIGNAL_PIN HIGH for same ON duration
static inline void blinkWithSignal(uint16_t ms = 50) {
  digitalWrite(SIGNAL_PIN, HIGH); // start external signal
  ledWrite(true);                 // LED ON (active-low aware)
  delay(ms);
  ledWrite(false);                // LED OFF
  digitalWrite(SIGNAL_PIN, LOW);  // end external signal
}

// ====== Exact payload match helper ======
static inline bool payload_equals(const char* data, uint16_t len, const char* s) {
  size_t n = strlen(s);
  if (len != n) return false;
  return memcmp(data, s, n) == 0;
}
// === END COMMON BLOCK ===


// ====== Receiver implementation ======
struct_package inPkg;

void onRecv(RECV_CB_ARGS) {
  if (len < (int)sizeof(struct_package)) {
    Serial.printf("Receiver: short packet (%d bytes)\n", len);
    return;
  }
  memcpy(&inPkg, data, sizeof(inPkg));

  // Duplicate check
  if (cache_contains(inPkg.uuid)) return;
  cache_insert(inPkg.uuid);

  Serial.printf("Receiver: got package \"%.*s\" (UUID: %u%u%u%u)\n", inPkg.length, inPkg.data, inPkg.uuid[0], inPkg.uuid[1], inPkg.uuid[2], inPkg.uuid[3]);

  // Match logic: exactly "reboot"
  if (payload_equals(inPkg.data, inPkg.length, "reboot")) {
    Serial.println("Match found!");
    blinkWithSignal(600);
  } else {
    Serial.println("No match.");
    blink(50);
  }
}

Servo monServo;

void setup() {
  Serial.begin(115200);
  ledInit();
  signalInit();

  WiFi.mode(WIFI_STA);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);
}

void loop() {
  // idle
}
