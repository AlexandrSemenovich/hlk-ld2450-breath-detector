#pragma once
// Bridge that streams breath results to the PC over the USB UART.
// Keep this minimal — it must not block the real-time loop.

#include <Arduino.h>
#include "breath_detector.h"

namespace bridge {

// Compact, low-latency stream protocol (one line per message):
//   D<dist>,<ac>,<depth>,<lateral>,<visible>,<in_zone>,<stationary>,<ts_ms>,<frame_id>
//   S<bpm>,<amp>,<qual>,<visible>,<in_zone>,<stationary>,<ts_ms>,<frame_id>
// The 'D'/'S' prefix lets the PC demultiplex without CSV ambiguity.
// dist/ac/depth/lateral are in mm. ts_ms is millis() on the ESP, frame_id
// monotonically increases so the PC can detect dropped frames.

// Send one sample (called per radar frame -> minimal latency).
inline void sendSample(float dist_mm, float ac_mm, float depth_mm, float lateral_mm,
                       bool visible, bool in_zone, bool stationary,
                       uint32_t ts_ms, uint32_t frame_id) {
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
  Serial.print(stationary ? 1 : 0);
  Serial.print(',');
  Serial.print(ts_ms);
  Serial.print(',');
  Serial.println(frame_id);
}

// Send the periodic summary line.
inline void sendSummary(const breath::Result& r, bool visible, bool in_zone, bool stationary,
                        uint32_t ts_ms, uint32_t frame_id) {
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
  Serial.print(stationary ? 1 : 0);
  Serial.print(',');
  Serial.print(ts_ms);
  Serial.print(',');
  Serial.println(frame_id);
}

inline void begin(uint32_t baud) {
  Serial.begin(baud);
  Serial.println("ESP32 LD2450 breath detector ready");
  Serial.println("protocol: D<dist>,<ac>,<depth>,<lateral>,<visible>,<in_zone>,<stationary>,<ts_ms>,<frame_id> per sample");
  Serial.println("protocol: S<bpm>,<amp>,<qual>,<visible>,<in_zone>,<stationary>,<ts_ms>,<frame_id> per summary");
}

}  // namespace bridge
