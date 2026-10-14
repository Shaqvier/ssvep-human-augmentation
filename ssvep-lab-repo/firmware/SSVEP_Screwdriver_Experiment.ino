/*
 * SSVEP_Screwdriver_Experiment.ino
 * Cued SSVEP stimulus sketch for the screwdriver task (2026-07-26 version).
 *
 * Matches the serial protocol used by ros2_nodes/wrist_jog_screwdriver.py:
 *   's' -> start trial (begin cue + stimulus sequence)
 *   'w' -> cued class 0 (unscrew, 19 Hz)
 *   'x' -> cued class 1 (screw, 23 Hz)
 *   'h' -> host says "moving home" (blank/idle stimulus, wait)
 *   'e' -> host says "at home" (ready for next trial)
 *
 * Per-trial sequence: receive cue byte -> flicker the corresponding LED
 * strip segment at its target frequency for STIM_MS -> blank -> wait for
 * 'h' (host moving to home) -> wait for 'e' (host confirms at home) ->
 * ready for next 's'.
 *
 * Requires FastLED. LED strip: WS2812B, one strip split into two logical
 * segments (unscrew / screw) so each flickers independently.
 */

#include <FastLED.h>

#define LED_PIN     6
#define NUM_LEDS    60
#define NUM_HALF    (NUM_LEDS / 2)

CRGB leds[NUM_LEDS];

const float FREQS_HZ[2] = {19.0, 23.0};   // {unscrew, screw}
const unsigned long STIM_MS = 5000;       // 5 s trials

void setup() {
  Serial.begin(115200);
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();
}

char waitForByte(char want) {
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == want) return c;
    }
  }
}

void flickerSegment(int segment, float freqHz, unsigned long durationMs) {
  int start = (segment == 0) ? 0 : NUM_HALF;
  int end   = (segment == 0) ? NUM_HALF : NUM_LEDS;

  unsigned long periodUs = (unsigned long)(1000000.0 / freqHz);
  unsigned long halfPeriodUs = periodUs / 2;
  unsigned long t0 = millis();
  bool on = false;

  while (millis() - t0 < durationMs) {
    on = !on;
    for (int i = start; i < end; i++) {
      leds[i] = on ? CRGB::White : CRGB::Black;
    }
    FastLED.show();
    delayMicroseconds(halfPeriodUs);
  }

  for (int i = start; i < end; i++) leds[i] = CRGB::Black;
  FastLED.show();
}

void loop() {
  waitForByte('s');   // start trial

  char cue = '\0';
  while (cue != 'w' && cue != 'x') {
    if (Serial.available() > 0) cue = Serial.read();
  }
  int segment = (cue == 'w') ? 0 : 1;
  float freq = FREQS_HZ[segment];

  flickerSegment(segment, freq, STIM_MS);

  waitForByte('h');   // host moving to home: stay blank
  FastLED.clear();
  FastLED.show();

  waitForByte('e');   // host confirms at home: ready for next trial
}
