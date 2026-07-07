#pragma once
// LD2450 UART frame parser.
// Protocol frame (per datasheet):
//   AA FF 03 00 | nTargets | reserved | 8*nTargets | 55 CC
// Each target: x(2) y(2) speed(2) distance_res(2)
//   x, y      : signed int16, MSB = sign (1 -> positive), unit mm
//   speed     : signed int16, MSB = sign (1 -> positive), unit cm/s
//   distance_res : uint16, distance gate size (mm); 0 means target absent
// A missing/unused target slot is filled with all zeros (distance_res == 0).

#include <cstdint>
#include <cstddef>

namespace ld2450 {

static constexpr uint8_t  MAX_TARGETS = 3;
static constexpr size_t   RX_CAPACITY = 64;

struct Target {
  int16_t x;
  int16_t y;
  int16_t speed;     // cm/s, signed
  uint16_t distance_res;  // mm, 0 => target not present
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

  // Drop all buffered bytes (used to recover from a desync).
  void reset() { len_ = 0; }

  // Number of buffered bytes not yet consumed.
  uint16_t size() const { return len_; }

  // Copy the raw bytes of the most recently extracted valid frame.
  uint16_t lastRaw(uint8_t* out, uint16_t maxlen) const;

 private:
  uint8_t  buf_[RX_CAPACITY];
  uint16_t len_ = 0;

  uint8_t  raw_[RX_CAPACITY];   // last extracted valid frame
  uint16_t raw_len_ = 0;

  static int16_t decodeCoord(uint16_t raw);
  static int16_t decodeSpeed(uint16_t raw);
};

}  // namespace ld2450
