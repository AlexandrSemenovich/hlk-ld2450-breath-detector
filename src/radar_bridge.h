#pragma once
// Bridge that streams raw radar target frames to the PC over the USB UART.
// The ESP32 is a transparent forwarder: it only parses the LD2450 wire frame
// and relays the raw targets. All target selection + breath analysis happens
// on the PC (python/monitor.py). Keep this minimal — it must not block the loop.

#include <Arduino.h>
#include "ld2450_parser.h"

namespace bridge {

// Compact, low-latency stream protocol (one line per radar frame):
//   R<x0>,<y0>,<spd0>,<res0>,<x1>,<y1>,<spd1>,<res1>,
//     <x2>,<y2>,<spd2>,<res2>,<ts_ms>,<frame_id>
// x/y  : signed mm (lateral x, longitudinal y)
// spd  : signed cm/s
// res  : uint16, distance gate resolution mm (0 => target slot unused)
// ts_ms: millis() on the ESP. frame_id monotonically increases so the PC can
//        detect dropped frames.
// No breath processing is done here — the PC does target selection and
// detection from these raw coordinates.

inline void sendTargets(const ld2450::Frame& f, uint32_t ts_ms, uint32_t frame_id) {
  Serial.print('R');
  for (uint8_t i = 0; i < ld2450::MAX_TARGETS; i++) {
    const auto& t = f.targets[i];
    Serial.print(static_cast<int>(t.x));
    Serial.print(',');
    Serial.print(static_cast<int>(t.y));
    Serial.print(',');
    Serial.print(static_cast<int>(t.speed));
    Serial.print(',');
    Serial.print(static_cast<unsigned int>(t.distance_res));
    Serial.print(',');
  }
  Serial.print(ts_ms);
  Serial.print(',');
  Serial.println(frame_id);
}

inline void begin(uint32_t baud) {
  Serial.begin(baud);
  Serial.println("ESP32 LD2450 breath detector ready");
  Serial.println(
      "protocol: R<x0>,<y0>,<spd0>,<res0>,<x1>,<y1>,<spd1>,<res1>,"
      "<x2>,<y2>,<spd2>,<res2>,<ts_ms>,<frame_id> per target frame");
}

}  // namespace bridge
