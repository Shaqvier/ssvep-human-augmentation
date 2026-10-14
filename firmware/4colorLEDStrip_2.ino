#include <FastLED.h>

#define NUM_LEDS 36
#define LED_PIN 3
#define BRIGHTNESS 150
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

// ── SSVEP frequencies (Hz) ───────────────────────────────────────────────
// Off-alpha (8-13 Hz avoided), fully harmonic-clean: no fundamental and no
// harmonic (up to 3rd) coincides across the set. Non-uniform spacing (3/4/6)
// avoids even-spacing intermodulation pile-up.
//   16 -> 32, 48 | 19 -> 38, 57 | 23 -> 46, 69 | 29 -> 58, 87
const float FREQS_HZ[4] = {16.0f, 19.0f, 23.0f, 29.0f};

// Optional per-group phase offset (radians). Leave at 0 unless you need
// phase-coded disambiguation. Phase coding does NOT mitigate peripheral
// SSVEP leakage (an amplitude effect) — it only helps decoder-side.
const float PHASES_RAD[4] = {0.0f, 0.0f, 0.0f, 0.0f};

const CRGB baseColors[4] = {CRGB::Red, CRGB::Blue, CRGB::Green, CRGB::Yellow};

// Half-period in microseconds, computed at runtime as 500000 / f. Float edge
// targets prevent integer-truncation phase drift over long runs.
float halfPeriod[4];
double nextToggle[4];   // floating-point edge target, microseconds
uint8_t states[4] = {0, 0, 0, 0};

void setup() {
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear(true);

  double now = (double)micros();
  for (uint8_t i = 0; i < 4; i++) {
    halfPeriod[i] = 500000.0f / FREQS_HZ[i];
    // Apply phase offset as a fraction of a half-period.
    double phaseShift = (PHASES_RAD[i] / (2.0 * PI)) * (2.0 * halfPeriod[i]);
    nextToggle[i] = now + phaseShift;
  }
}

void loop() {
  double now = (double)micros();
  bool dirty = false;

  for (uint8_t i = 0; i < 4; i++) {
    if (now >= nextToggle[i]) {
      nextToggle[i] += halfPeriod[i];   // float edge target — avoids drift
      states[i] = !states[i];
      CRGB color = states[i] ? baseColors[i] : CRGB::Black;
      fill_solid(leds + (i * 9), 9, color);
      dirty = true;
    }
  }

  if (dirty) {
    FastLED.show();
  }
}
