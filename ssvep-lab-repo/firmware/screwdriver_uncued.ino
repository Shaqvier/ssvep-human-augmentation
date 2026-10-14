#include <FastLED.h>

// ── LED STRIP ─────────────────────────────────────────────────────────────────
#define NUM_LEDS    16
#define LED_PIN      3
#define BRIGHTNESS 150
#define LED_TYPE  WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

// ── FREQUENCIES ───────────────────────────────────────────────────────────────
//   Class 0 → Red  → 16 Hz → unscrew (left)
//   Class 1 → Blue → 19 Hz → screw   (right)
const float FREQS_HZ[2]    = {16.0f, 19.0f};
const CRGB  BASE_COLORS[2] = {CRGB::Red, CRGB::Green};
const uint8_t GROUP_START[2] = {0, 12};   // LED index of each group's first pixel
const uint8_t GROUP_SIZE      = 4;        // LEDs per group

// ── EEG TRIGGER ───────────────────────────────────────────────────────────────
// Pin 9, same as offline test. Pulse HIGH for 5 ms at cue onset.
// Trigger value sent matches class index (0 or 1) so EEG records trial type.
#define TRIGGER_PIN  9
#define TRIGGER_MS   5

// ── TIMING (ms) ───────────────────────────────────────────────────────────────
#define CUE_MS   1500   // solid colour on; trigger fires at onset
#define STIM_MS  60000   // both colours flickering; robot active
#define BLANK_MS 1000   // all off; inter-trial interval

// ── TRIAL SEQUENCE ────────────────────────────────────────────────────────────
// 10 trials: 5× class 0 (Red/unscrew) + 5× class 1 (Blue/screw).
// Balanced, no two adjacent trials the same class.
const uint8_t SEQUENCE[10] = {0, 1, 0, 1, 0, 1, 1, 0, 1, 0};
const uint8_t N_TRIALS = 10;

// ── FLICKER STATE ─────────────────────────────────────────────────────────────
float   halfPeriod[2];
double  nextToggle[2];
uint8_t flickerState[2];

// ── HELPERS ───────────────────────────────────────────────────────────────────

void allOff() {
    FastLED.clear();
    FastLED.show();
}

// Pulse trigger pin HIGH for TRIGGER_MS then LOW.
// The value argument is sent over Serial so the PC/EEG host can log trial type.
void sendTrigger(uint8_t value) {
    digitalWrite(TRIGGER_PIN, HIGH);
    delay(TRIGGER_MS);
    digitalWrite(TRIGGER_PIN, LOW);
    // Also echo over serial for logging / BrainVision marker injection
    Serial.print("TRIGGER ");
    Serial.println(value);
}

// Solid cue: only the target group lit, all others off.
void showCue(uint8_t cls) {
    FastLED.clear();
    fill_solid(leds + GROUP_START[cls], GROUP_SIZE, BASE_COLORS[cls]);
    FastLED.show();
}

// Reset edge timers so both groups start flickering in sync at call time.
void initFlicker() {
    double now = (double)micros();
    for (uint8_t i = 0; i < 2; i++) {
        halfPeriod[i]   = 500000.0f / FREQS_HZ[i];
        nextToggle[i]   = now;
        flickerState[i] = 0;
    }
    FastLED.clear();
    FastLED.show();
}

// Run both groups flickering for durationMs milliseconds.
void runFlicker(unsigned long durationMs) {
    unsigned long startMs = millis();
    while (millis() - startMs < durationMs) {
        double now = (double)micros();
        bool dirty = false;
        for (uint8_t i = 0; i < 2; i++) {
            if (now >= nextToggle[i]) {
                nextToggle[i] += halfPeriod[i];
                flickerState[i] = !flickerState[i];
                CRGB color = flickerState[i] ? BASE_COLORS[i] : CRGB::Black;
                fill_solid(leds + GROUP_START[i], GROUP_SIZE, color);
                dirty = true;
            }
        }
        if (dirty) FastLED.show();
    }
}

// ── SETUP ─────────────────────────────────────────────────────────────────────

void setup() {
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
    allOff();

    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);

    Serial.begin(115200);
    Serial.println("SSVEP screwdriver experiment ready.");
    Serial.println("  Class 0 = Red  (16 Hz) = unscrew left");
    Serial.println("  Class 1 = Blue (19 Hz) = screw right");
    Serial.println("Send 'g' to start block.");
}

// ── MAIN LOOP ─────────────────────────────────────────────────────────────────

void loop() {
    if (!Serial.available()) return;
    char cmd = Serial.read();
    if (cmd != 'g') return;

    Serial.println("Starting block...");

    for (uint8_t t = 0; t < N_TRIALS; t++) {
        uint8_t cls = SEQUENCE[t];

        Serial.print("Trial ");
        Serial.print(t + 1);
        Serial.print("/");
        Serial.print(N_TRIALS);
        Serial.print("  class=");
        Serial.print(cls);
        Serial.print("  freq=");
        Serial.print((int)FREQS_HZ[cls]);
        Serial.print(" Hz  action=");
        Serial.println(cls == 0 ? "UNSCREW" : "SCREW");

        // ── 1. CUE (1.5 s): solid colour + EEG trigger ────────────────────────
//        showCue(cls);
//        sendTrigger(cls);                        // trigger value = class index
//        delay(CUE_MS - TRIGGER_MS);             // remainder of cue window

        // ── 2. STIMULUS (60 s): both colours flicker; send 's' to gate robot ───
        initFlicker();
        Serial.write('s');                       // ROS node: start accepting jogs
        while(1){
          runFlicker(STIM_MS);
        }
        Serial.write('x');                       // ROS node: stop accepting jogs

        // ── 3. BLANK (1 s): all off ────────────────────────────────────────────
//        allOff();
//        delay(BLANK_MS);
    }

    Serial.println("Block complete.");
    allOff();
}
