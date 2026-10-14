#include <FastLED.h>

#define NUM_LEDS 36
#define LED_PIN 3
#define BRIGHTNESS 150
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

// Half-period intervals in microseconds for each group
// 18 Hz -> 27778 us, 15 Hz -> 33333 us, 12 Hz -> 41667 us, 8 Hz -> 62500 us
const unsigned long intervals[4] = {27778, 33333, 41667, 62500};
const CRGB baseColors[4] = {CRGB::Red, CRGB::Blue, CRGB::Green, CRGB::Yellow};

unsigned long lastToggle[4] = {0, 0, 0, 0};
uint8_t states[4] = {0, 0, 0, 0};

void setup() {
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear(true);

  unsigned long now = micros();
  for (uint8_t i = 0; i < 4; i++) {
    lastToggle[i] = now;
  }
}

void loop() {
  unsigned long now = micros();
  bool dirty = false;

  for (uint8_t i = 0; i < 4; i++) {
    if (now - lastToggle[i] >= intervals[i]) {
      lastToggle[i] += intervals[i];  // preserves phase, avoids drift
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
