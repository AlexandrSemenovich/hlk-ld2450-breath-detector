#include "ld2450_parser.h"
#include <cstring>

namespace ld2450 {

void Parser::feed(uint8_t b) {
  if (len_ < RX_CAPACITY) {
    buf_[len_++] = b;
  } else {
    memmove(buf_, buf_ + 1, RX_CAPACITY - 1);
    buf_[RX_CAPACITY - 1] = b;
  }
}

int16_t Parser::decodeSigned(uint16_t raw) {
    // Signed-magnitude: bit15 is the sign (1 => positive, 0 => negative).
    const int16_t mag = static_cast<int16_t>(raw & 0x7FFF);
    return (raw & 0x8000) ? mag : static_cast<int16_t>(-mag);
}

bool Parser::drain(Frame& out) {
  out.valid = false;
  if (len_ < 30) return false;

  if (buf_[0] != 0xAA || buf_[1] != 0xFF || buf_[2] != 0x03 || buf_[3] != 0x00 ||
      buf_[28] != 0x55 || buf_[29] != 0xCC) {
    memmove(buf_, buf_ + 1, --len_);
    return false;
  }

  out.count = 0;
  for (uint8_t i = 0; i < MAX_TARGETS; i++) {
    const uint16_t o = 4 + i * 8;
    Target& t = out.targets[i];

    t.index = i;
    t.x = decodeSigned(buf_[o] | (buf_[o+1] << 8));
    t.y = decodeSigned(buf_[o+2] | (buf_[o+3] << 8));
    t.speed = decodeSigned(buf_[o+4] | (buf_[o+5] << 8));
    t.distance_res = buf_[o+6] | (buf_[o+7] << 8);

    // A target slot is "present" only when distance_res != 0 (datasheet).
    if (t.distance_res != 0) {
      out.count++;
    }
  }

  out.valid = true;
  len_ = 0;
  return true;
}

}  // namespace ld2450