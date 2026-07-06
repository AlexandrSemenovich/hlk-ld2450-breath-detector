#pragma once
// LD2450 UART frame parser.
// Protocol frame (per datasheet):
//   AA FF 03 00 | nTargets | reserved | 8*nTargets | 55 CC
// Each target: x(2) y(2) speed(2) reserved(2)  -> signed 16-bit coords.

#include <cstdint>
#include <cstddef>

namespace ld2450 {

static constexpr uint8_t  MAX_TARGETS = 3;
static constexpr size_t   RX_CAPACITY = 64;

struct Target {
  int16_t x;
  int16_t y;
  uint16_t speed;
  uint16_t reserved;
  uint8_t  index;   // order in frame
};

struct Frame {
  Target  targets[MAX_TARGETS];
  uint8_t count = 0;
  bool    valid = false;
};

// Accumulates bytes and extracts complete frames.
// Call feed() with every received byte, then drain() until it returns false.
class Parser {
 public:
  // Feed one received byte into the internal buffer.
  void feed(uint8_t b);

  // Try to extract one complete frame from the buffer.
  // Returns true and fills `out` when a frame was consumed.
  bool drain(Frame& out);

 private:
  uint8_t  buf_[RX_CAPACITY];
  uint16_t len_ = 0;

  static int16_t decodeCoord(uint16_t raw);
};

}  // namespace ld2450
