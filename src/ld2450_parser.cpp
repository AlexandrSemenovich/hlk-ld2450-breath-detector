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

// Per datasheet: signed int16, MSB (0x8000) is the sign bit.
//   MSB set   -> positive value, magnitude = raw & 0x7FFF
//   MSB clear -> negative value, magnitude = raw & 0x7FFF
// Example: 0x0E03 = 782 -> negative -> -782 mm
//          0x86B1 = 34481 -> positive -> 34481 - 32768 = 1713 mm
int16_t Parser::decodeCoord(uint16_t raw) {
  const int16_t mag = static_cast<int16_t>(raw & 0x7FFF);
  return (raw & 0x8000) ? mag : static_cast<int16_t>(-mag);
}

// Speed shares the same sign encoding as coordinates (unit cm/s).
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

  // Number of targets. The real module emits a fixed 3-target frame; some
  // bytes at this position can be garbage on a desync, so clamp to a valid
  // range to avoid an absurd frame length (which would stall the parser).
  uint8_t nTargets = buf_[4];
  if (nTargets < 1 || nTargets > MAX_TARGETS) nTargets = MAX_TARGETS;

  uint16_t need = static_cast<uint16_t>(6 + nTargets * 8);  // hdr+len+reserved+targets+tail
  if (len_ < need) return false;                            // wait for full frame

  // Tail: 55 CC
  if (buf_[need - 2] != 0x55 || buf_[need - 1] != 0xCC) {
    memmove(buf_, buf_ + 1, --len_);
    return false;
  }

  for (uint8_t i = 0; i < nTargets; i++) {
    uint16_t o = static_cast<uint16_t>(6 + i * 8);
    Target& t = out.targets[i];
    t.x           = decodeCoord(static_cast<uint16_t>(buf_[o]     | (buf_[o + 1] << 8)));
    t.y           = decodeCoord(static_cast<uint16_t>(buf_[o + 2] | (buf_[o + 3] << 8)));
    t.speed       = decodeSpeed(static_cast<uint16_t>(buf_[o + 4] | (buf_[o + 5] << 8)));
    t.distance_res = static_cast<uint16_t>(buf_[o + 6] | (buf_[o + 7] << 8));
    t.index       = i;
  }
  out.count = nTargets;
  out.valid = true;

  // Keep a copy of the raw frame for diagnostics.
  raw_len_ = (need <= RX_CAPACITY) ? need : RX_CAPACITY;
  memcpy(raw_, buf_, raw_len_);

  memmove(buf_, buf_ + need, len_ - need);
  len_ -= need;
  return true;
}

uint16_t Parser::lastRaw(uint8_t* out, uint16_t maxlen) const {
  const uint16_t n = (raw_len_ < maxlen) ? raw_len_ : maxlen;
  if (n && out) memcpy(out, raw_, n);
  return raw_len_;
}

}  // namespace ld2450
