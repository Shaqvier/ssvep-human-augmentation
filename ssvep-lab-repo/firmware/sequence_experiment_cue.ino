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

// ── EEG TRIGGER ──────────────────────────────────────────────────────────────
#define TRIGGER_PIN   9
#define TRIGGER_MS    5           // standard EEG marker pulse width (ms)

// ── TIMING (ms) ──────────────────────────────────────────────────────────────
#define CUE_MS      1500          // solid cue colour; trigger fires at onset
#define BLANK_MS    1000          // all off, after robot completes a move
#define FLICKER_TIMEOUT_MS 10000  // max flicker before auto-advance (classifier miss)

// ── TRIAL SEQUENCE ───────────────────────────────────────────────────────────
//   0 = Red (16 Hz)   1 = Blue (19 Hz)   2 = Green (23 Hz)   3 = Yellow (29 Hz)
// Balanced, no adjacent repeats. Runs ONCE per 'g' then stops ("Block complete."
// + 'x' sentinel). Send 'g' again to re-run the same sequence as a new block.
//const uint8_t SEQUENCE[20] = {
//    0, 2, 1, 3, 2,
//    0, 3, 1, 2, 3,
//    1, 0, 3, 2, 0,
//    3, 1, 2, 0, 1
//};
//const uint8_t N_TRIALS = 20;

const uint8_t SEQUENCE[4] = {
    0, 2, 1, 3
};
const uint8_t N_TRIALS = 4;

// ── STATE ────────────────────────────────────────────────────────────────────
float   halfPeriod[4];
double  nextToggle[4];
uint8_t flickerState[4];

uint8_t trialIdx = 0;             // position in SEQUENCE
bool    running  = false;         // set true by 'g'

// Experiment/block counter. Increments on every 'g' (block start); resets to 0
// only on Arduino reset/flash. Forwarded to Python as "EXP:<n>" so the
// experiment-agnostic MATLAB recorder can name each continuous recording.
uint16_t experimentId = 0;

// Phases within one trial
enum Phase { CUE, FLICKER, BLANK, IDLE };
Phase phase = IDLE;

unsigned long phaseStartMs = 0;

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

void showCue(uint8_t groupIdx) {
    FastLED.clear();
    fill_solid(leds + (groupIdx * 9), 9, BASE_COLORS[groupIdx]);
    FastLED.show();
}

void initFlicker() {
    double now = (double)micros();
    for (uint8_t i = 0; i < 4; i++) {
        halfPeriod[i]   = 500000.0f / FREQS_HZ[i];
        nextToggle[i]   = now;
        flickerState[i] = 0;
    }
    FastLED.clear();
    FastLED.show();
}

// One pass of the flicker update (non-blocking; call every loop iteration).
void stepFlicker() {
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

// Begin the CUE phase for the current trial.
void startTrial() {
    // Sequence exhausted -> block complete. Stop and wait for the next 'g'.
    // This mirrors the offline sketch (finite for-loop over N_TRIALS) rather
    // than wrapping modulo N_TRIALS forever. Emitting 'x' here means the
    // MATLAB recorder flushes automatically -- no need to send 'e' by hand.
    if (trialIdx >= N_TRIALS) {
        running = false;
        phase   = IDLE;
        allOff();
        Serial.println("x");              // end-of-block sentinel -> recorder
        Serial.println("Block complete.");
        return;
    }

    uint8_t cls = SEQUENCE[trialIdx];

    Serial.print("Trial ");
    Serial.print(trialIdx + 1);
    Serial.print("/");
    Serial.print(N_TRIALS);
    Serial.print("  class=");
    Serial.print(cls);
    Serial.print("  freq=");
    Serial.print((int)FREQS_HZ[cls]);
    Serial.println(" Hz");

    showCue(cls);
    sendTrigger();                    // EEG trigger at cue onset (TTL, pin 9)
    // Machine-parseable cue marker for the MATLAB fallback path (used when the
    // pin-9 TTL is NOT visible in the LSL stream). Emitted right at cue onset,
    // immediately after the hardware trigger, so the two coincide.
    Serial.print("CUE:");
    Serial.println(cls);
    phase = CUE;
    phaseStartMs = millis();
}

// ── SETUP ────────────────────────────────────────────────────────────────────
void setup() {
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
    allOff();

    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);

    Serial.begin(115200);
    Serial.println("ROS-gated SSVEP ready. Send 'g' to start, 'e' to end block.");
}

// ── MAIN LOOP ─────────────────────────────────────────────────────────────────
void loop() {

    // ── Serial command handler ──────────────────────────────────────────────
    if (Serial.available()) {
        char cmd = Serial.read();

        if (cmd == 'g') {                       // start the whole block
            trialIdx = 0;
            running  = true;
            experimentId++;                     // new block -> new experiment id
            Serial.print("EXP:");               // forwarded to MATLAB via Python
            Serial.println(experimentId);
            Serial.println("Block started.");
            startTrial();

        } else if (cmd == 'e') {                // end block/experiment cleanly
            running = false;
            phase   = IDLE;
            allOff();
            Serial.println("x");                // end-of-block sentinel -> MATLAB
            Serial.println("Block ended.");

        } else if (cmd == 'p') {                // robot moving -> freeze LEDs
            if (running) {
                running = false;
                phase   = IDLE;
                allOff();
                Serial.println("Paused (robot moving).");
            }

        } else if (cmd == 'r') {                // robot done -> next trial
            if (!running) {
                running  = true;
                trialIdx++;                     // advance to next colour
                Serial.println("Resumed. Next trial.");
                startTrial();
            }

        } else if (cmd == 'c') {                // scan failed -> repeat SAME trial
            // NOTE: the robot node no longer sends 'c' — on scan failure it now
            // sends 'r' (advance to next colour). This handler is retained for
            // manual use from the Serial Monitor and to make reverting trivial.
            if (!running) {
                running = true;                 // no trialIdx++ : same colour
                Serial.println("Resumed (same trial).");
                startTrial();
            }
        }
    }

    if (!running) return;

    // ── Phase state machine ─────────────────────────────────────────────────
    unsigned long elapsed = millis() - phaseStartMs;

    switch (phase) {
        case CUE:
            if (elapsed >= CUE_MS) {
                initFlicker();
                phase = FLICKER;
                phaseStartMs = millis();
            }
            break;

        case FLICKER:
            // Flicker until 'p' arrives (MATLAB gets 4-in-a-row) OR 10s elapses.
            if (elapsed >= FLICKER_TIMEOUT_MS) {
                Serial.println("t");        // notify ROS: classifier timeout/miss
                allOff();
                trialIdx++;                 // advance to NEXT colour (option A)
                phase = BLANK;
                phaseStartMs = millis();
            } else {
                stepFlicker();
            }
            break;

        case BLANK:
            if (elapsed >= BLANK_MS) {
                startTrial();               // loops back to CUE
            }
            break;

        case IDLE:
        default:
            break;
    }
}
