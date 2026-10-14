#include <FastLED.h>

// ── LED STRIP ────────────────────────────────────────────────────────────────
#define NUM_LEDS     36
#define LED_PIN       3
#define BRIGHTNESS  150
#define LED_TYPE  WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

// ── SSVEP FREQUENCIES ────────────────────────────────────────────────────────
// 16/19/23/29 Hz: off-alpha, no harmonic clashes to 3rd, non-uniform spacing.
const float FREQS_HZ[4]    = {16.0f, 19.0f, 23.0f, 29.0f};
const CRGB  BASE_COLORS[4] = {CRGB::Red, CRGB::Blue, CRGB::Green, CRGB::Yellow};

// ── EEG TRIGGER PIN (from TwoButtons.ino) ────────────────────────────────────
// Pulse this HIGH for TRIGGER_MS at the START of each cue phase.
// Wire to your actiCHamp digital trigger input.
#define TRIGGER_PIN   9
#define TRIGGER_MS    5       // ms; standard EEG marker pulse width

// ── TIMING (ms) ──────────────────────────────────────────────────────────────
#define CUE_MS      1500      // solid colour on, trigger fires at onset
#define STIM_MS     4000      // all four colours flickering
#define BLANK_MS    1000      // all off between trials

// ── TRIAL SEQUENCE ───────────────────────────────────────────────────────────
// 20 trials: each of the 4 class indices (0-3) appears exactly 5 times.
// Balanced so no colour repeats back-to-back, and the run is deterministic.
// Generated offline with a constrained shuffle (verified: no adjacent repeats).
//   0 = Red  (16 Hz)
//   1 = Blue (19 Hz)
//   2 = Green(23 Hz)
//   3 = Yellow(29 Hz)
const uint8_t SEQUENCE[20] = {
    0, 2, 1, 3, 2,   // trials  1-5
    0, 3, 1, 2, 3,   // trials  6-10
    1, 0, 3, 2, 0,   // trials 11-15
    3, 1, 2, 0, 1    // trials 16-20
};
const uint8_t N_TRIALS = 20;

// ── FLICKER STATE ─────────────────────────────────────────────────────────────
float  halfPeriod[4];
double nextToggle[4];
uint8_t flickerState[4];

// ── HELPERS ──────────────────────────────────────────────────────────────────

void allOff() {
    FastLED.clear();
    FastLED.show();
}

void sendTrigger() {
    digitalWrite(TRIGGER_PIN, HIGH);
    delay(TRIGGER_MS);
    digitalWrite(TRIGGER_PIN, LOW);
}

// Show a solid (non-flickering) colour for one group; all others off.
void showCue(uint8_t groupIdx) {
    FastLED.clear();
    fill_solid(leds + (groupIdx * 9), 9, BASE_COLORS[groupIdx]);
    FastLED.show();
}

// Initialise flicker edge timers so all groups start in sync.
void initFlicker() {
    double now = (double)micros();
    for (uint8_t i = 0; i < 4; i++) {
        halfPeriod[i]  = 500000.0f / FREQS_HZ[i];
        nextToggle[i]  = now;    // all start at onset
        flickerState[i] = 0;
    }
    FastLED.clear();
    FastLED.show();
}

// Run the flicker loop for durationMs milliseconds (non-blocking style via
// unsigned long elapsed; avoids micros() rollover issues for short durations).
void runFlicker(unsigned long durationMs) {
    unsigned long startMs = millis();
    while (millis() - startMs < durationMs) {
        double now = (double)micros();
        bool dirty = false;
        for (uint8_t i = 0; i < 4; i++) {
            if (now >= nextToggle[i]) {
                nextToggle[i] += halfPeriod[i];
                flickerState[i] = !flickerState[i];
                CRGB color = flickerState[i] ? BASE_COLORS[i] : CRGB::Black;
                fill_solid(leds + (i * 9), 9, color);
                dirty = true;
            }
        }
        if (dirty) FastLED.show();
    }
}

// ── SETUP ────────────────────────────────────────────────────────────────────

void setup() {
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
    allOff();

    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);

    Serial.begin(115200);
    Serial.println("SSVEP offline experiment ready.");
    Serial.println("Send 'g' to start.");
}

// ── MAIN LOOP ─────────────────────────────────────────────────────────────────

void loop() {
    // Wait for 'g' over serial before starting the block.
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
        Serial.println(" Hz");

        // ── 1. CUE (1 s): solid colour + EEG trigger ──
        showCue(cls);
        sendTrigger();          // pulse at cue onset
        delay(CUE_MS - TRIGGER_MS);   // remainder of cue period

        // ── 2. STIMULUS (4 s): all four colours flickering ──
        initFlicker();
        runFlicker(STIM_MS);

        // ── 3. BLANK (1 s): all off ──
        allOff();
        delay(BLANK_MS);
    }

    Serial.println("Block complete.");
    allOff();
}
