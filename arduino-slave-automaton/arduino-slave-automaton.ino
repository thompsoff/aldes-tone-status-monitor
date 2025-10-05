/*
  Arduino Uno: Pulse-triggered servo sequence with startup move + console logs
  - Startup: servo -> DOWN, wait 10s, servo -> UP
  - Trigger (D2 RISING): DOWN, wait 'SEQUENCE_WAIT_MS', UP
  - Pulses ignored while a sequence is running
  - Servo is detached after 'HOLD_MS' to avoid heating/wear
  - Verbose Serial logs for each step/state
*/

#include <Servo.h>

// ---------------------- User-configurable parameters ----------------------

// Pins (Uno): D2 supports external interrupt INT0
const uint8_t PULSE_PIN = 2;   // input pin for trigger pulse
const uint8_t SERVO_PIN = 9;   // PWM-capable pin for the servo signal

// Servo angles
const int SERVO_DOWN_ANGLE = 10;  // "down" position (0..180)
const int SERVO_UP_ANGLE   = 165; // "up" position   (0..180)

// Timing (all in milliseconds)
const unsigned long HOLD_MS             = 2000UL;   // how long to keep servo powered after each move
const unsigned long STARTUP_UP_DELAY_MS = 5 * 1000UL - HOLD_MS; // 10 seconds after boot to go UP
const unsigned long SEQUENCE_WAIT_MS    = 20 * 60 * 1000UL - HOLD_MS;  // wait between DOWN and UP during triggered sequence

// Input configuration
const bool USE_PULLUP = false; // true -> pinMode(INPUT_PULLUP) and expect LOW->HIGH from open contact
// --------------------------------------------------------------------------

enum State {
  BOOT_DOWN,         // just commanded DOWN; waiting to finish HOLD_MS then detach
  BOOT_WAIT,         // counting 10s before commanding UP
  BOOT_UP_HOLD,      // commanded UP; holding for HOLD_MS then detach → IDLE
  IDLE,              // waiting for pulses
  SEQ_DOWN_HOLD,     // triggered: commanded DOWN; hold for HOLD_MS then detach
  SEQ_WAIT,          // waiting SEQUENCE_WAIT_MS while detached
  SEQ_UP_HOLD        // commanded UP; hold for HOLD_MS then detach → IDLE
};

volatile bool pulseFlag = false;  // set by ISR on rising edge, consumed in loop when IDLE
Servo s;
State state;
unsigned long t0 = 0;             // timestamp for state timing
bool servoAttached = false;

// ------------------------------ Utilities ---------------------------------
void logMsg(const char* msg) {
  // Small helper to prefix logs with time
  Serial.print('[');
  Serial.print(millis());
  Serial.print(" ms] ");
  Serial.println(msg);
}

void attachIfNeeded() {
  if (!servoAttached) {
    s.attach(SERVO_PIN);
    servoAttached = true;
    logMsg("Servo ATTACH");
  }
}
void detachIfNeeded() {
  if (servoAttached) {
    s.detach();
    servoAttached = false;
    logMsg("Servo DETACH");
  }
}
void moveServo(int angle, const char* label) {
  attachIfNeeded();
  angle = constrain(angle, 0, 180);
  s.write(angle);
  t0 = millis(); // mark time for HOLD_MS
  Serial.print('['); Serial.print(t0); Serial.print(" ms] ");
  Serial.print("MOVE -> ");
  Serial.print(label);
  Serial.print(" (");
  Serial.print(angle);
  Serial.println("°)");
}
// --------------------------------------------------------------------------

// ISR must be fast—don’t print here.
void onPulse() {
  pulseFlag = true;
}

void setup() {
  Serial.begin(115200);
  // optional: wait a moment for the Serial Monitor to open (comment out for production)
  delay(50);
  logMsg("Booting...");

  if (USE_PULLUP) {
    pinMode(PULSE_PIN, INPUT_PULLUP);
    logMsg("PULSE_PIN set to INPUT_PULLUP (expect LOW->HIGH on release)");
  } else {
    pinMode(PULSE_PIN, INPUT);
    logMsg("PULSE_PIN set to INPUT (expect external HIGH pulse)");
  }

  // Interrupt on rising edge of D2
  attachInterrupt(digitalPinToInterrupt(PULSE_PIN), onPulse, RISING);
  logMsg("Interrupt attached on RISING edge (D2)");

  // // Startup sequence: go to DOWN immediately, hold briefly, then wait 10s, then go UP
  // moveServo(SERVO_DOWN_ANGLE, "DOWN (startup)");
  // state = BOOT_DOWN;
  // logMsg("STATE -> BOOT_DOWN");

  // Skip the DOWN position on startup and go directly to UP 
  moveServo(SERVO_UP_ANGLE, "UP (startup)");
  state = BOOT_UP_HOLD;
}

void loop() {
  unsigned long now = millis();

  switch (state) {

    case BOOT_DOWN:
      if (now - t0 >= HOLD_MS) {
        detachIfNeeded();
        t0 = now;           // start 10s delay
        state = BOOT_WAIT;
        logMsg("STATE -> BOOT_WAIT (waiting 10s before UP)");
      }
      break;

    case BOOT_WAIT:
      if (now - t0 >= STARTUP_UP_DELAY_MS) {
        moveServo(SERVO_UP_ANGLE, "UP (startup)");
        state = BOOT_UP_HOLD;
        logMsg("STATE -> BOOT_UP_HOLD");
      }
      break;

    case BOOT_UP_HOLD:
      if (now - t0 >= HOLD_MS) {
        detachIfNeeded();
        state = IDLE;
        logMsg("STATE -> IDLE (ready for trigger)");
      }
      break;

    case IDLE:
      if (pulseFlag) {
        pulseFlag = false;             // consume it
        logMsg("Pulse detected in IDLE -> start sequence");
        moveServo(SERVO_DOWN_ANGLE, "DOWN");
        state = SEQ_DOWN_HOLD;
        logMsg("STATE -> SEQ_DOWN_HOLD");
      }
      break;

    case SEQ_DOWN_HOLD:
      if (now - t0 >= HOLD_MS) {
        detachIfNeeded();
        t0 = now;                      // start inter-move wait
        state = SEQ_WAIT;
        Serial.print('['); Serial.print(now); Serial.print(" ms] ");
        Serial.print("WAIT between DOWN and UP: ");
        Serial.print(SEQUENCE_WAIT_MS);
        Serial.println(" ms");
        logMsg("STATE -> SEQ_WAIT");
      }
      break;

    case SEQ_WAIT:
      if (now - t0 >= SEQUENCE_WAIT_MS) {
        moveServo(SERVO_UP_ANGLE, "UP");
        state = SEQ_UP_HOLD;
        logMsg("STATE -> SEQ_UP_HOLD");
      }
      break;

    case SEQ_UP_HOLD:
      if (now - t0 >= HOLD_MS) {
        detachIfNeeded();
        state = IDLE; // ready for the next pulse
        logMsg("Sequence complete. STATE -> IDLE");
      }
      break;
  }

  // Any pulses while not IDLE are ignored; ensure no backlog:
  if (state != IDLE && pulseFlag) {
    pulseFlag = false; // drop it
    logMsg("Pulse occurred during sequence -> ignored");
  }
}
