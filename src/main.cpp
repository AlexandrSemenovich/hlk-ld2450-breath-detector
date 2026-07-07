#include <Arduino.h>
#include <HardwareSerial.h>

#include "ld2450_parser.h"
#include "breath_detector.h"
#include "radar_bridge.h"

// Hardware config
static constexpr uint8_t  LD2450_RX_PIN = 32;
static constexpr uint8_t  LD2450_TX_PIN = 33;
static constexpr uint32_t LD2450_BAUD   = 256000;
static constexpr uint32_t PC_BAUD       = 921600;

#ifndef DEBUG
#define DEBUG 1
#endif

// Radar reports at ~10 Hz (datasheet). Used only for coarse guards.
static constexpr uint32_t FRAME_TIMEOUT_MS = 1000;
static constexpr uint32_t SUMMARY_MS = 500;

// Target validity / selection
static constexpr float    VALID_R_MIN = 100.0f;
static constexpr float    VALID_R_MAX = 6000.0f;
static constexpr float    ZONE_R_MIN  = 600.0f;
static constexpr float    ZONE_R_MAX  = 2500.0f;
static constexpr float    ZONE_SIDE_MAX = 500.0f;
static constexpr float    STATIONARY_SPEED_THRESHOLD = 80.0f;  // cm/s

HardwareSerial radarSerial(2);

static ld2450::Parser   parser;
static breath::Detector detector({});
static int16_t          locked_index = -1;
static uint32_t         last_frame_ms = 0;
static uint32_t         frame_counter = 0;

// Helpers
static float safe_sqrt(float x) { return x > 0 ? sqrtf(x) : 0.0f; }

// Radial (breath) distance from the radar to the target.
static float radial(const ld2450::Target& t) {
  return safe_sqrt(static_cast<float>(t.x) * t.x + static_cast<float>(t.y) * t.y);
}

// Lateral (cross-range) component — signed x axis.
static float lateralOf(const ld2450::Target& t) {
  return static_cast<float>(t.x);
}

static bool isPresent(const ld2450::Target& t) {
  float r = radial(t);
  return (r >= VALID_R_MIN && r <= VALID_R_MAX);
}

static bool inZone(const ld2450::Target& t) {
  float r = radial(t);
  return (r >= ZONE_R_MIN && r <= ZONE_R_MAX && fabsf(t.x) <= ZONE_SIDE_MAX);
}

// Pick the best target for breathing analysis:
//   - prefer the previously locked target if still valid and stationary in zone
// Selection priority:
//   1) keep the previously locked target if still breath-ready
//   2) closest stationary target inside the zone (breath-ready)
//   3) fallback: closest present target (so the PC can show/aim it)
static int16_t pickTarget(const ld2450::Frame& f, int16_t locked) {
  if (locked >= 0) {
    for (uint8_t i = 0; i < f.count; i++) {
      const auto& t = f.targets[i];
      if (t.index == locked && isPresent(t) && inZone(t) &&
          fabsf(t.speed) < STATIONARY_SPEED_THRESHOLD) {
        return i;
      }
    }
  }

  int16_t best = -1;
  float best_score = 1e12f;
  for (uint8_t i = 0; i < f.count; i++) {
    const auto& t = f.targets[i];
    if (!isPresent(t) || !inZone(t)) continue;
    if (fabsf(t.speed) >= STATIONARY_SPEED_THRESHOLD) continue;  // need a still chest
    float score = radial(t);
    if (score < best_score) {
      best_score = score;
      best = i;
    }
  }
  if (best >= 0) return best;

  // Fallback: nearest present target (used for aiming/debugging on the PC).
  best = -1;
  best_score = 1e12f;
  for (uint8_t i = 0; i < f.count; i++) {
    const auto& t = f.targets[i];
    if (!isPresent(t)) continue;
    float score = radial(t);
    if (score < best_score) {
      best_score = score;
      best = i;
    }
  }
  return best;
}

static void pollRadar(uint32_t now) {
  if (parser.size() > 0 && (now - last_frame_ms) > FRAME_TIMEOUT_MS) {
    parser.reset();
  }

  while (radarSerial.available()) {
    parser.feed(radarSerial.read());
  }

  ld2450::Frame f;
  while (parser.drain(f)) {
    if (!f.valid) continue;

    last_frame_ms = now;

    int16_t sel = pickTarget(f, locked_index);

    if (sel >= 0) {
      const auto& t = f.targets[sel];
      float depth     = radial(t);                 // breath axis (mm)
      float lateral   = lateralOf(t);              // cross-range (mm)
      bool  stationary = fabsf(t.speed) < STATIONARY_SPEED_THRESHOLD;
      bool  in_zone    = inZone(t);

      // Only feed the breath detector with a still target inside the zone.
      if (in_zone && stationary) {
        detector.push(depth, now);
      } else {
        detector.reset();
      }

      bridge::sendSample(depth, detector.ac(), depth, lateral,
                         true, in_zone, stationary, now, ++frame_counter);
      locked_index = t.index;
      continue;
    }

    // No suitable target.
    if (locked_index != -1) {
      detector.reset();
      locked_index = -1;
    }
    bridge::sendSample(0.0f, 0.0f, 0.0f, 0.0f,
                       false, false, false, now, ++frame_counter);
  }
}

void setup() {
  bridge::begin(PC_BAUD);
  radarSerial.begin(LD2450_BAUD, SERIAL_8N1, LD2450_RX_PIN, LD2450_TX_PIN);
  Serial.println("ESP32 LD2450 Breath Detector v3 - transparent forwarder");
}

void loop() {
  pollRadar(millis());

  static uint32_t last_summary = 0;
  uint32_t now = millis();
  if (now - last_summary >= SUMMARY_MS) {
    last_summary = now;
    bridge::sendSummary(detector.update(now), false, false, false, now, frame_counter);
  }
}
