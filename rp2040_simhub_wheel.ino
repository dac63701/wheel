/*
 * RP2040 SimHub steering wheel firmware — discrete single-colour LEDs version
 * ---------------------------------------------------------------------------
 * Hardware: 10x separate single-colour LEDs + 2 quadrature encoders (no push
 * switches) + 4 buttons, all on one RP2040 over USB.
 *
 * SimHub integration (no new protocol invented):
 *   Implements the documented SimHub "Standard Serial" SIMHUB_1.0 subset:
 *     (FF FF FF FF FF FF) + "proto" -> "SIMHUB_1.0\r\n"
 *     (FF FF FF FF FF FF) + "ledsc" -> "10\r\n"
 *     (FF FF FF FF FF FF) + "sleds" + 10*RGB + (FF FE FD) -> latch LEDs, no reply
 *   Serial: USB CDC, 115200 8N1. DTR/RTS are ignored.
 *
 * IMPORTANT consequence of discrete single-colour LEDs: the firmware is an
 * ON/OFF framebuffer. Each sleds slot's RGB triple is reduced to on/off via a
 * brightness threshold (max(R,G,B) >= LED_ON_THRESHOLD). SimHub colour/threshold
 * configuration still works, but colours collapse to on/off and only the
 * brightness/threshold information survives. Shift-flash works because SimHub
 * blinks the frames themselves.
 *
 * Inputs:
 *   USB HID gamepad (composite CDC+HID) for direct game use, plus optional
 *   human-readable serial event lines for CUSTOM PC software only
 *   (stock SimHub does not parse these event lines).
 *
 * Board/core:
 *   Earle Philhower arduino-pico core, Tools -> USB Stack: "Pico SDK"
 *   (gives Serial CDC + Joystick HID composite). No LED library needed.
 *
 * GPIO map (RP2040 GP numbers, 3V3 logic):
 *   GP6..GP15        -> LEDs 1..10 (GPIO -> series resistor -> LED anode,
 *                        LED cathode -> GND; active-HIGH; LED 1 = leftmost
 *                        from driver-seat view, SimHub maps RPM left-to-right)
 *   GP2 / GP3        -> Encoder 1 A / B (common to GND, INPUT_PULLUP)
 *   GP4 / GP5        -> Encoder 2 A / B (common to GND, INPUT_PULLUP)
 *   GP18..GP21       -> Buttons 1..4 (to GND, INPUT_PULLUP, active-low)
 *   GP25             -> onboard LED (status/heartbeat)
 *   GP0/GP1 left free for UART debug. Avoid GP23/24/25 for inputs.
 *   USB D+/D- are dedicated. GP16/17/22/26/27/28 remain free for expansion.
 *   Total used: 10 LED + 4 encoder + 4 button = 18 GPIOs.
 *
 * LED drive note: each GPIO drives one LED through its own resistor.
 *   Red (~2.0 Vf): 330R => ~4 mA. Green/yellow/blue/white (higher Vf):
 *   220R => ~3-6 mA. This keeps per-pin current well under the 12 mA limit
 *   and total draw modest. For high-brightness LEDs use low-side NPN/ULN2803
 *   drivers instead of direct drive (firmware unchanged, still active-HIGH).
 */

// Joystick HID is bundled with the arduino-pico core when USB stack = Pico SDK.
// If your core/menu lacks it, set ENABLE_HID to 0: LEDs + serial still work.
#define ENABLE_HID 1
#if ENABLE_HID && __has_include(<Joystick.h>)
#include <Joystick.h>
#define HAVE_JOYSTICK 1
#else
#define HAVE_JOYSTICK 0
#endif

// ------------------------------ config ------------------------------------
#define NUM_LEDS 10
const uint8_t LED_PINS[NUM_LEDS] = { 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
#define LED_ACTIVE_HIGH 1   // 1 = GPIO HIGH lights LED (anode via resistor to GPIO)
#define LED_ON_THRESHOLD 24 // max(R,G,B) >= this => ON (SimHub dims/off fall below)
#define LED_TIMEOUT_MS 0    // 0 = hold last frame; e.g. 5000 to blank on timeout

#define ENC1_A 2
#define ENC1_B 3
#define ENC2_A 4
#define ENC2_B 5

#define NUM_BUTTONS 4
const uint8_t BTN_PINS[NUM_BUTTONS] = { 18, 19, 20, 21 };

#define BTN_DEBOUNCE_MS 20
#define ENC_PULSE_MS 80     // HID pulse length per detent
#define ENC_DEBOUNCE_US 1500

// Serial event lines for custom integration. Default OFF so the SimHub
// proto/ledsc scanner never sees interleaved async text.
// Enable by sending ASCII "events1\n" (disable with "events0\n").
#define DEFAULT_EVENTS_ENABLED 0

// HID mapping (1-based for Joystick.button()):
//   1..4  = buttons 1..4 (level)
//   5/6   = ENC1 CCW/CW (momentary pulse), 7/8 = ENC2 CCW/CW
#define HID_BTN_BASE 1
#define HID_ENC1_CCW 5
#define HID_ENC1_CW 6
#define HID_ENC2_CCW 7
#define HID_ENC2_CW 8

// ------------------------------ state -------------------------------------
static bool eventsEnabled = DEFAULT_EVENTS_ENABLED;
static uint32_t lastFrameMs = 0;
static bool frameReceived = false;
static bool ledState[NUM_LEDS];

struct DebouncedBtn {
  uint8_t pin;
  bool stable;  // true = pressed
  bool lastRaw;
  uint32_t lastChange;
};
static DebouncedBtn buttons[NUM_BUTTONS];

// Encoders: full-quadrature state machine, 4 transitions = 1 detent.
// (No push switches on these encoders: A/B quadrature only.)
struct Encoder {
  uint8_t pinA, pinB;
  uint8_t prevAB;
  int8_t accum;  // +/-4 => one detent
  uint32_t lastEdgeUs;
  uint32_t pulseUntil;
  uint8_t hidBtn;
  int dir;
};
static Encoder encoders[2];

enum ParseState : uint8_t { PS_SYNC, PS_CMD, PS_SLEDS_DATA };
static ParseState pstate = PS_SYNC;
static uint8_t syncCount = 0;
static char cmdBuf[5];
static uint8_t cmdLen = 0;
static uint8_t ledBuf[NUM_LEDS * 3];
static uint16_t ledIdx = 0;
static uint8_t termBuf[3];
static uint8_t termLen = 0;
static uint32_t frameStartMs = 0;

static const int8_t QTABLE[16] = {
   0, +1, -1,  0,
  -1,  0,  0, +1,
  +1,  0,  0, -1,
   0, -1, +1,  0
};

// ------------------------------ helpers -----------------------------------
static inline uint8_t readAB(Encoder &e) {
  uint8_t a = digitalRead(e.pinA);
  uint8_t b = digitalRead(e.pinB);
  return (uint8_t)((a << 1) | b);
}

static void hidSet(uint8_t btn, bool pressed) {
#if HAVE_JOYSTICK
  Joystick.button(btn, pressed);
#else
  (void)btn; (void)pressed;
#endif
}

static inline void writeLed(uint8_t i, bool on) {
  ledState[i] = on;
#if LED_ACTIVE_HIGH
  digitalWrite(LED_PINS[i], on ? HIGH : LOW);
#else
  digitalWrite(LED_PINS[i], on ? LOW : HIGH);
#endif
}

static void latchLeds() {
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    uint8_t r = ledBuf[i * 3], g = ledBuf[i * 3 + 1], b = ledBuf[i * 3 + 2];
    uint8_t m = r > g ? r : g;
    m = m > b ? m : b;
    writeLed(i, m >= LED_ON_THRESHOLD);
  }
  lastFrameMs = millis();
  frameReceived = true;
}

static void allLeds(bool on) {
  for (uint8_t i = 0; i < NUM_LEDS; i++) writeLed(i, on);
}

static void resetParser() {
  pstate = PS_SYNC; syncCount = 0; cmdLen = 0; ledIdx = 0; termLen = 0;
}

// Self-test: chase, then all-on, then off. Proves every LED + order.
static void selfTest() {
  allLeds(false);
  for (uint8_t i = 0; i < NUM_LEDS; i++) { writeLed(i, true); delay(80); writeLed(i, false); }
  allLeds(true); delay(300);
  allLeds(false);
}

// ------------------------------ SimHub parser ------------------------------
static void parseByte(uint8_t b) {
  uint32_t now = millis();
  switch (pstate) {
    case PS_SYNC:
      if (b == 0xFF) {
        if (++syncCount == 6) { pstate = PS_CMD; cmdLen = 0; frameStartMs = now; }
      } else {
        syncCount = 0;
      }
      break;
    case PS_CMD:
      if (cmdLen < 5) {
        cmdBuf[cmdLen++] = (char)b;
        if (cmdLen == 5) {
          if (memcmp(cmdBuf, "proto", 5) == 0) {
            Serial.print("SIMHUB_1.0\r\n");
            resetParser();
          } else if (memcmp(cmdBuf, "ledsc", 5) == 0) {
            Serial.print(NUM_LEDS);
            Serial.print("\r\n");
            resetParser();
          } else if (memcmp(cmdBuf, "sleds", 5) == 0) {
            ledIdx = 0; termLen = 0; pstate = PS_SLEDS_DATA;
          } else {
            resetParser();
          }
        }
      }
      if (now - frameStartMs > 100) resetParser();
      break;
    case PS_SLEDS_DATA:
      if (ledIdx < sizeof(ledBuf)) {
        ledBuf[ledIdx++] = b;
      } else if (termLen < 3) {
        termBuf[termLen++] = b;
        if (termLen == 3) {
          if (termBuf[0] == 0xFF && termBuf[1] == 0xFE && termBuf[2] == 0xFD) latchLeds();
          resetParser();
        }
      }
      if (now - frameStartMs > 100) resetParser();
      break;
  }
}

// ------------------------------ inputs ------------------------------------
static void pollButtons(uint32_t now) {
  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    DebouncedBtn &bt = buttons[i];
    bool raw = (digitalRead(bt.pin) == LOW);
    if (raw != bt.lastRaw) { bt.lastRaw = raw; bt.lastChange = now; }
    if ((now - bt.lastChange) >= BTN_DEBOUNCE_MS && raw != bt.stable) {
      bt.stable = raw;
      hidSet((uint8_t)(HID_BTN_BASE + i), raw);
      if (eventsEnabled) {
        Serial.print("BTN ");
        Serial.print(i + 1);
        Serial.println(raw ? " PRESSED" : " RELEASED");
      }
    }
  }
}

static void encoderDetent(Encoder &e, int dir, int idx, uint32_t now) {
  e.dir = dir;
  uint8_t hid = (idx == 0) ? (dir > 0 ? HID_ENC1_CW : HID_ENC1_CCW)
                           : (dir > 0 ? HID_ENC2_CW : HID_ENC2_CCW);
  if (e.hidBtn && e.hidBtn != hid) hidSet(e.hidBtn, false);
  e.hidBtn = hid;
  hidSet(hid, true);
  e.pulseUntil = now + ENC_PULSE_MS;
  if (eventsEnabled) {
    Serial.print("ENC ");
    Serial.print(idx + 1);
    Serial.println(dir > 0 ? " +1" : " -1");
  }
}

static void pollEncoders(uint32_t now) {
  uint32_t nowUs = micros();
  for (uint8_t i = 0; i < 2; i++) {
    Encoder &e = encoders[i];
    uint8_t cur = readAB(e);
    if (cur != e.prevAB) {
      if (nowUs - e.lastEdgeUs >= ENC_DEBOUNCE_US) {
        e.accum += QTABLE[(uint8_t)((e.prevAB << 2) | cur) & 0x0F];
        e.prevAB = cur;
        e.lastEdgeUs = nowUs;
        if (e.accum >= 4) { e.accum = 0; encoderDetent(e, +1, i, now); }
        else if (e.accum <= -4) { e.accum = 0; encoderDetent(e, -1, i, now); }
      }
    }
    if (e.hidBtn && (int32_t)(now - e.pulseUntil) >= 0) {
      hidSet(e.hidBtn, false);
      e.hidBtn = 0;
    }
  }
}

// ------------------------------ setup/loop --------------------------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);

  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    pinMode(LED_PINS[i], OUTPUT);
#if LED_ACTIVE_HIGH
    digitalWrite(LED_PINS[i], LOW);
#else
    digitalWrite(LED_PINS[i], HIGH);
#endif
    ledState[i] = false;
  }

#if HAVE_JOYSTICK
  Joystick.begin(false);
#endif

  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    buttons[i].pin = BTN_PINS[i];
    pinMode(BTN_PINS[i], INPUT_PULLUP);
    bool raw = (digitalRead(BTN_PINS[i]) == LOW);
    buttons[i].stable = raw; buttons[i].lastRaw = raw;
    buttons[i].lastChange = millis();
    hidSet((uint8_t)(HID_BTN_BASE + i), raw);
  }

  encoders[0] = { ENC1_A, ENC1_B, 0, 0, 0, 0, 0, 0 };
  encoders[1] = { ENC2_A, ENC2_B, 0, 0, 0, 0, 0, 0 };
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(encoders[i].pinA, INPUT_PULLUP);
    pinMode(encoders[i].pinB, INPUT_PULLUP);
    encoders[i].prevAB = readAB(encoders[i]);
    encoders[i].lastEdgeUs = micros();
  }

  selfTest();  // visual proof all 10 LEDs + order work before SimHub connects
  digitalWrite(LED_BUILTIN, LOW);
  lastFrameMs = millis();
}

static char lineBuf[32];
static uint8_t lineLen = 0;

static void handleLine(const char *line) {
  if (strcmp(line, "events1") == 0) { eventsEnabled = true; Serial.println("EVENTS ON"); }
  else if (strcmp(line, "events0") == 0) { eventsEnabled = false; Serial.println("EVENTS OFF"); }
  else if (strcmp(line, "test") == 0) { selfTest(); Serial.println("TEST DONE"); }
  else if (strcmp(line, "ping") == 0) { Serial.println("pong"); }
}

void loop() {
  uint32_t now = millis();
  while (Serial.available()) {
    int c = Serial.read();
    if (c < 0) break;
    uint8_t b = (uint8_t)c;
    bool inFrame = (pstate != PS_SYNC) || (b == 0xFF);
    if (inFrame) {
      parseByte(b);
      lineLen = 0;
    } else {
      parseByte(b);
      if (b == '\n') {
        uint8_t n = lineLen < sizeof(lineBuf) - 1 ? lineLen : sizeof(lineBuf) - 1;
        lineBuf[n] = '\0';
        if (n && lineBuf[n - 1] == '\r') lineBuf[n - 1] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      } else if (lineLen < sizeof(lineBuf) - 1) {
        lineBuf[lineLen++] = (char)b;
      } else {
        lineLen = 0;
      }
    }
  }

#if LED_TIMEOUT_MS
  if (frameReceived && (now - lastFrameMs > LED_TIMEOUT_MS)) {
    allLeds(false);
    frameReceived = false;
  }
#else
  (void)frameReceived; (void)lastFrameMs;
#endif

  pollButtons(now);
  pollEncoders(now);

#if HAVE_JOYSTICK
  Joystick.send_now();
#endif

  digitalWrite(LED_BUILTIN, (now / 1000) % 2 == 0 ? HIGH : LOW);
}
