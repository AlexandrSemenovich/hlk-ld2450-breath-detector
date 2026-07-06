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

int16_t Parser::decodeCoord(uint16_t raw) {
  if (raw & 0x8000) return static_cast<int16_t>(raw - 0x8000);
  return -static_cast<int16_t>(raw);
}

bool Parser::drain(Frame& out) {
  out.valid = false;
  if (len_ < 4) return false;

  // Header: AA FF 03 00
  if (buf_[0] != 0xAA || buf_[1] != 0xFF || buf_[2] != 0x03 || buf_[3] != 0x00) {
    memmove(buf_, buf_ + 1, --len_);   // resync by one byte
    return false;
  }

  uint8_t nTargets = buf_[4];
  if (nTargets > MAX_TARGETS) nTargets = MAX_TARGETS;
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
    t.x        = decodeCoord(static_cast<uint16_t>(buf_[o]     | (buf_[o + 1] << 8)));
    t.y        = decodeCoord(static_cast<uint16_t>(buf_[o + 2] | (buf_[o + 3] << 8)));
    t.speed    = static_cast<uint16_t>(buf_[o + 4] | (buf_[o + 5] << 8));
    t.reserved = static_cast<uint16_t>(buf_[o + 6] | (buf_[o + 7] << 8));
    t.index    = i;
  }
  out.count = nTargets;
  out.valid = true;

  memmove(buf_, buf_ + need, len_ - need);
  len_ -= need;
  return true;
}

}  // namespace ld2450
