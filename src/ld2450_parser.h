#pragma once
// LD2450 UART frame parser.
// Real wire frame is a fixed 30 bytes:
//   AA FF 03 00 (4) | 3 x [x(2) y(2) speed(2) distance_res(2)] (24) | 55 CC (2)
// i.e. first target starts at offset 4; there is NO separate nTargets/reserved byte.
//   x, y      : signed-magnitude int16 (magnitude = raw & 0x7FFF, sign in bit15), unit mm
//   speed     : signed-magnitude int16 (sign in bit15), unit cm/s
//   distance_res : uint16, distance gate size (mm); 0 means target absent
// A missing/unused target slot is filled with all zeros (distance_res == 0).

#include <cstdint>
#include <cstddef>

namespace ld2450 {

static constexpr uint8_t  MAX_TARGETS = 3;
static constexpr size_t   RX_CAPACITY = 64;

#define ZONE_MAX_Y  3500   // Максимальная дальность прямо перед радаром (например, 3.5 метра)
#define ZONE_MIN_Y  500    // Минимальная дальность (обычно 0)
#define ZONE_MAX_X  2000   // Максимальное отклонение вправо (2 метра)
#define ZONE_MIN_X -2000   // Максимальное отклонение влево (2 метра)

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

 private:
  uint8_t  buf_[RX_CAPACITY];
  uint16_t len_ = 0;

  static int16_t decodeCoord(uint16_t raw);
  static int16_t decodeSpeed(uint16_t raw);
};

}  // namespace ld2450
