#include "ld2450_parser.h"
#include <cstring>

namespace ld2450 {

void Parser::feed(uint8_t b) {
  if (len_ < RX_CAPACITY) {
    buf_[len_++] = b;
  } else {
    // Overflow guard: drop oldest byte and keep resyncing.
    memmove(buf_, buf_ + 1, RX_CAPACITY - 1);
    buf_[RX_CAPACITY - 1] = b;
  }
}

// LD2450 emits x, y and speed using signed-magnitude encoding:
//   magnitude = raw & 0x7FFF, sign in bit15 (0x8000).
// IMPORTANT: a plain int16_t(raw) cast is WRONG here — it turns valid
// coordinates into huge out-of-range values and breaks target selection
// (everything decodes to 0 / no target). This signed-magnitude decode is
// symmetric (bit15=1 -> positive, bit15=0 -> negative); a target on the
// opposite side of the sensor flips the sign. If your mounting mirrors
// left/right, flip the sign in both functions (one line each).
int16_t Parser::decodeCoord(uint16_t raw) {
  const int16_t mag = static_cast<int16_t>(raw & 0x7FFF);
  return (raw & 0x8000) ? mag : static_cast<int16_t>(-mag);
}

// Speed uses the same signed-magnitude encoding (unit cm/s).
int16_t Parser::decodeSpeed(uint16_t raw) {
  const int16_t mag = static_cast<int16_t>(raw & 0x7FFF);
  return (raw & 0x8000) ? mag : static_cast<int16_t>(-mag);
}

bool Parser::drain(Frame& out) {
  out.valid = false;
  if (len_ < 4) return false;

  // Header: AA FF 03 00
  if (buf_[0] != 0xAA || buf_[1] != 0xFF || buf_[2] != 0x03 || buf_[3] != 0x00) {
    memmove(buf_, buf_ + 1, --len_);   // resync by one byte
    return false;
  }

  // The LD2450 emits a fixed 30-byte frame: 4-byte header, then 3 targets
  // (8 bytes each, first target at offset 4), then a 55 CC tail. There is NO
  // separate nTargets/reserved byte on the wire — this matches LD2450.cpp.
  const uint8_t nTargets = MAX_TARGETS;                          // 3
  const uint16_t need = static_cast<uint16_t>(4 + nTargets * 8 + 2);  // 30
  if (len_ < need) return false;                                // wait for full frame

  // Tail: 55 CC
  if (buf_[need - 2] != 0x55 || buf_[need - 1] != 0xCC) {
    memmove(buf_, buf_ + 1, --len_);
    return false;
  }

  for (uint8_t i = 0; i < nTargets; i++) {
    const uint16_t o = static_cast<uint16_t>(4 + i * 8);        // first target at offset 4
    Target& t = out.targets[i];

    t.x           = decodeCoord(static_cast<uint16_t>(buf_[o]     | (buf_[o + 1] << 8)));
    t.y           = decodeCoord(static_cast<uint16_t>(buf_[o + 2] | (buf_[o + 3] << 8)));
    t.speed       = decodeSpeed(static_cast<uint16_t>(buf_[o + 4] | (buf_[o + 5] << 8)));
    t.distance_res = static_cast<uint16_t>(buf_[o + 6] | (buf_[o + 7] << 8));
    t.index       = i;
  }
  out.count = nTargets;
  out.valid = true;

  memmove(buf_, buf_ + need, len_ - need);
  len_ -= need;
  return true;
}

}  // namespace ld2450
