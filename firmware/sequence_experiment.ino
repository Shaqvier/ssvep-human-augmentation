#include <FastLED.h>

// ── LED STRIP ────────────────  ────────────────────────────────────────────────
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

// ────────────────────────────────────────────────────────────────────────────
// VERBAL-CUE, FREE-RUNNING PARADIGM
// ────────────────────────────────────────────────────────────────────────────
// No LED cue phase and no fixed trial sequence. The experimenter names the
// target aloud; the participant looks at it. The four groups flicker
// CONTINUOUSLY between robot moves. There is no per-trial colour assignment and
// no trial count — the block free-runs until 'e'.
//
// Serial protocol (unchanged bytes, new meaning where noted):
//   g : start block  -> new experimentId, begin flickering
//   p : robot moving -> freeze LEDs (all off)
//   r : robot home   -> fire pin-9 TTL (timing ref) + resume flickering
//   e : end block    -> stop, emit 'x' sentinel (recorder flushes)
//
// The pin-9 TTL now marks ROBOT-RETURNED-HOME, not cue onset. The Python node
// also publishes a 'home_reached' ROS marker on the same event.

// ── STATE ────────────────────────────────────────────────────────────────────
float   halfPeriod[4];
double  nextToggle[4];
uint8_t flickerState[4];

bool    running  = false;         // true while flickering (set by 'g'/'r')

// Experiment/block counter. Increments on every 'g'; resets only on reset/flash.
// Forwarded to Python as "EXP:<n>" so the recorder can name each recording.
uint16_t experimentId = 0;

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

// ── SETUP ────────────────────────────────────────────────────────────────────
void setup() {
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
    allOff();

    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);

    Serial.begin(115200);
    Serial.println("Verbal-cue SSVEP ready. 'g' start, 'p' pause, 'r' home, 'e' end.");
}

// ── MAIN LOOP ─────────────────────────────────────────────────────────────────
void loop() {

    // ── Serial command handler ──────────────────────────────────────────────
    if (Serial.available()) {
        char cmd = Serial.read();

        if (cmd == 'g') {                       // start block: begin flickering
            experimentId++;
            Serial.print("EXP:");               // forwarded to MATLAB via Python
            Serial.println(experimentId);
            Serial.println("Block started.");
            initFlicker();
            running = true;

        } else if (cmd == 'e') {                // end block
            running = false;
            allOff();
            Serial.println("x");                // end-of-block sentinel -> MATLAB
            Serial.println("Block ended.");

        } else if (cmd == 'p') {                // robot moving -> freeze LEDs
            if (running) {
                running = false;
                allOff();
                Serial.println("Paused (robot moving).");
            }

        } else if (cmd == 'r') {                // robot home -> TTL + resume
            // Fire the pin-9 TTL to mark ROBOT-RETURNED-HOME on the EEG, then
            // resume continuous flickering for the next selection.
            sendTrigger();                      // EEG timing reference (home)
            Serial.println("HOME");             // parseable marker -> Python
            if (!running) {
                initFlicker();
                running = true;
                Serial.println("Resumed (flickering).");
            }
        }
    }

    if (!running) return;

    // Continuous flicker — no phases, no cue, no sequence.
    stepFlicker();
}
