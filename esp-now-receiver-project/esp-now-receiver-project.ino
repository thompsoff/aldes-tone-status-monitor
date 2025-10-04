#include <WiFi.h>
#include <esp_now.h>

// Pins
#define LED_PIN    8   // Builtin LED on ESP32-C3
#define PULSE_PIN  2   // Output pulse pin

// Set to true if your builtin LED is active-LOW (common on ESP32-C3 devkits).
// If your LED is active-HIGH, set this to false.
constexpr bool LED_ACTIVE_LOW = true;

// Helper to handle LED polarity
inline void LED_ON()  { digitalWrite(LED_PIN,  LED_ACTIVE_LOW ? LOW  : HIGH); }
inline void LED_OFF() { digitalWrite(LED_PIN,  LED_ACTIVE_LOW ? HIGH : LOW); }

// Deadlines for turning outputs off; 0 means "inactive"
volatile unsigned long ledOffAtMs   = 0;
volatile unsigned long pulseOffAtMs = 0;

// Called whenever an ESP-NOW packet is received
void onReceive(const esp_now_recv_info_t* recv_info,
               const uint8_t* data,
               int len) {
  // LED: ON for 200 ms
  LED_ON();
  ledOffAtMs = millis() + 200;

  // GPIO2: HIGH pulse for 20 ms (active-HIGH)
  digitalWrite(PULSE_PIN, HIGH);
  pulseOffAtMs = millis() + 20;
}

void setup() {
  // Configure outputs and ensure they start OFF
  pinMode(LED_PIN, OUTPUT);
  LED_OFF();

  pinMode(PULSE_PIN, OUTPUT);
  digitalWrite(PULSE_PIN, LOW);   // idle LOW, only HIGH during pulse

  // Enable Wi-Fi radio for ESP-NOW (no connection needed)
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW and register the RX callback
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onReceive);
  }
}

void loop() {
  // Non-blocking timeouts for turning outputs back OFF
  unsigned long now = millis();

  if (ledOffAtMs && (long)(now - ledOffAtMs) >= 0) {
    LED_OFF();
    ledOffAtMs = 0;
  }

  if (pulseOffAtMs && (long)(now - pulseOffAtMs) >= 0) {
    digitalWrite(PULSE_PIN, LOW);
    pulseOffAtMs = 0;
  }
}
