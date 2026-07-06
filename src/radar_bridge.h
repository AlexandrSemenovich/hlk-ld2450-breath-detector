#pragma once
// Bridge that streams breath results to the PC over the USB UART.
// Keep this minimal — it must not block the real-time loop.

#include <Arduino.h>
#include "breath_detector.h"

namespace bridge {

// Prints one CSV line: <bpm>,<amp_mm>,<quality>,<dist_mm>
inline void sendResult(const breath::Result& r) {
  Serial.print(r.bpm, 1);
  Serial.print(',');
  Serial.print(r.amplitude, 1);
  Serial.print(',');
  Serial.print(static_cast<int>(r.quality));
  Serial.print(',');
  Serial.println(r.distance, 1);
}

inline void begin(uint32_t baud) {
  Serial.begin(baud);
  delay(1000);
  Serial.println("ESP32 LD2450 breath detector ready");
  Serial.println("bpm,amp_mm,quality,dist_mm");
}

// Draws a scrolling ASCII waveform of the breath signal (AC, mm).
// `wave` holds the most-recent samples (oldest first). Zero is printed as
// a center line '|'; samples are mapped to height `half` characters.
inline void sendWave(const float* wave, uint16_t n, float scale_mm = 8.0f, uint8_t half = 12) {
  if (n == 0) return;
  for (uint16_t i = 0; i < n; ++i) {
    float v = wave[i] / scale_mm;        // normalize to ~[-1,1]
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    int16_t row = static_cast<int16_t>(v * half);   // signed offset from center
    for (int16_t r = -half; r <= half; ++r) {
      if (r == 0) {
        Serial.print('|');                 // zero baseline
      } else if (r == row) {
        Serial.print(row >= 0 ? '^' : 'v'); // sample marker (up/down)
      } else {
        Serial.print(' ');
      }
    }
    Serial.println();
  }
  Serial.println("---- breath waveform (^ inhale / v exhale) ----");
}

}  // namespace bridge
