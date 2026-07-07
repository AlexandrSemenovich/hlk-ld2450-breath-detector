#pragma once
// Bridge that streams breath results to the PC over the USB UART.
// Keep this minimal — it must not block the real-time loop.

#include <Arduino.h>
#include "breath_detector.h"

namespace bridge {

// Compact, low-latency stream protocol (one line per message):
//   D<dist_mm>,<ac_mm>,<depth_mm>,<lateral_mm>,<visible>,<in_zone>,<stationary>
//   S<bpm>,<amp>,<qual>,<visible>,<in_zone>,<stationary>
// The 'D'/'S' prefix lets the PC demultiplex without CSV ambiguity.

// Send one sample (called per radar frame -> minimal latency).
inline void sendSample(float dist_mm, float ac_mm, float depth_mm, float lateral_mm,
                       bool visible, bool in_zone, bool stationary) {
  Serial.print('D');
  Serial.print(dist_mm, 1);
  Serial.print(',');
  Serial.print(ac_mm, 2);
  Serial.print(',');
  Serial.print(depth_mm, 1);
  Serial.print(',');
  Serial.print(lateral_mm, 1);
  Serial.print(',');
  Serial.print(visible ? 1 : 0);
  Serial.print(',');
  Serial.print(in_zone ? 1 : 0);
  Serial.print(',');
  Serial.println(stationary ? 1 : 0);
}

// Send the periodic summary line.
inline void sendSummary(const breath::Result& r, bool visible, bool in_zone, bool stationary) {
  Serial.print('S');
  Serial.print(r.bpm, 1);
  Serial.print(',');
  Serial.print(r.amplitude, 1);
  Serial.print(',');
  Serial.print(static_cast<int>(r.quality));
  Serial.print(',');
  Serial.print(visible ? 1 : 0);
  Serial.print(',');
  Serial.print(in_zone ? 1 : 0);
  Serial.print(',');
  Serial.println(stationary ? 1 : 0);
}

inline void begin(uint32_t baud) {
  Serial.begin(baud);
  delay(1000);
  Serial.println("ESP32 LD2450 breath detector ready");
  Serial.println("protocol: D<dist>,<ac>,<depth>,<lateral>,<visible>,<in_zone>,<stationary> per sample");
  Serial.println("protocol: S<bpm>,<amp>,<qual>,<visible>,<in_zone>,<stationary> per summary");
}

}  // namespace bridge
