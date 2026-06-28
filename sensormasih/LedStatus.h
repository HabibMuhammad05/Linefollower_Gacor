#ifndef LED_STATUS_H
#define LED_STATUS_H
#include <Adafruit_NeoPixel.h>
#include "Config.h"
#include "SensorLine.h"

Adafruit_NeoPixel strip(NUM_LEDS, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

void ledInit() {
  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  strip.clear();
  strip.show();
}

void ledSolid(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

// Base: tiap LED = 1 sensor. Hijau = kena garis, merah redup = tidak.
void ledTampilkanSensor() {
  for (int i = 0; i < NUM_LEDS; i++) {
    if (sensorDigital[i]) {
      strip.setPixelColor(i, strip.Color(0, 60, 0));
    } else {
      strip.setPixelColor(i, strip.Color(8, 0, 0));
    }
  }
  strip.show();
}

void ledStandby() {
  static unsigned long timer = 0;
  static bool on = false;
  if (millis() - timer > 500) {
    timer = millis();
    on = !on;
    ledSolid(0, 0, on ? 40 : 0);
  }
}

void ledKalibrasi() {
  static uint8_t pos = 0;
  static unsigned long timer = 0;
  if (millis() - timer > 80) {
    timer = millis();
    strip.clear();
    strip.setPixelColor(pos, strip.Color(60, 60, 0));
    strip.show();
    pos = (pos + 1) % NUM_LEDS;
  }
}

void ledStop() {
  ledSolid(60, 0, 0);
}

#endif
