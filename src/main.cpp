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

static constexpr int16_t  MOTION_SPEED_CMS = 60;
static constexpr uint32_t FRAME_TIMEOUT_MS = 1000;
static constexpr uint32_t SUMMARY_MS = 500;
static constexpr float    STATIONARY_SPEED_THRESHOLD = 80.0f;


static constexpr float    VALID_R_MIN = 100.0f;
static constexpr float    VALID_R_MAX = 6000.0f;

static constexpr float    ZONE_R_MIN = 200.0f;
static constexpr float    ZONE_R_MAX = 2500.0f;
static constexpr float    ZONE_SIDE_MAX = 500.0f;

HardwareSerial radarSerial(2);

static ld2450::Parser      parser;
static ld2450::Frame       dbg_last_frame{};
static breath::Detector    detector({});
static int16_t             locked_index = -1;
static uint32_t            last_frame_ms = 0;
static uint32_t            last_summary_ms = 0;

// Debug
static uint32_t dbg_bytes = 0, dbg_frames = 0, dbg_samples = 0;

// Helpers
static float safe_sqrt(float x) { return x > 0 ? sqrtf(x) : 0.0f; }

static float radial(const ld2450::Target& t) {
  return safe_sqrt(static_cast<float>(t.x) * t.x + static_cast<float>(t.y) * t.y);
}

static float computeDistAndDepth(const ld2450::Target& t, float& out_depth) {
  float x = fabsf(static_cast<float>(t.x));   // берём модуль
  float y = static_cast<float>(t.y);
  float r = safe_sqrt(x*x + y*y);

  if (fabs(y) < 120.0f) {
    out_depth = r;                    // теперь почти = dist
  } else {
    out_depth = y;
  }

  // Защита от совсем плохих значений
  if (r < 150.0f) {
    r = 1000.0f;
    out_depth = 950.0f;
  }

  return r;
}

static bool isPresent(const ld2450::Target& t) {
  float r = radial(t);
  return (r >= VALID_R_MIN && r <= VALID_R_MAX);
}

static bool inZone(const ld2450::Target& t) {
  float r = radial(t);
  return (r >= ZONE_R_MIN && r <= ZONE_R_MAX && fabsf(t.x) <= ZONE_SIDE_MAX);
}

static int16_t pickTarget(const ld2450::Frame& f, int16_t locked) {
  if (locked >= 0) {
    for (uint8_t i = 0; i < f.count; i++) {
      if (f.targets[i].index == locked && isPresent(f.targets[i]) && inZone(f.targets[i])) {
        return i;
      }
    }
    return -1;
  }

  int16_t best = -1;
  float best_score = 1e12f;
  for (uint8_t i = 0; i < f.count; i++) {
    const auto& t = f.targets[i];
    if (!isPresent(t) || !inZone(t)) continue;
    float score = (abs(t.speed) < MOTION_SPEED_CMS) ? radial(t) : 1e9f;
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
    dbg_bytes++;
  }

  ld2450::Frame f;
  while (parser.drain(f)) {
    if (!f.valid) continue;
    
    last_frame_ms = now;
    dbg_frames++;
    dbg_last_frame = f;        // теперь должно работать

    // === Упрощённый выбор цели ===
    int16_t sel = -1;
    for (uint8_t i = 0; i < f.count; i++) {
      if (f.targets[i].x != 0 || f.targets[i].y != 0) {
        sel = i;
        break;
      }
    }

    if (sel >= 0) {
      const auto& t = f.targets[sel];
      float depth = static_cast<float>(t.x);
      float dist = computeDistAndDepth(t, depth);
      float lateral = static_cast<float>(t.y);

      bool stationary = fabsf(t.speed) < 150;

      detector.push(dist, now);

      bridge::sendSample(dist, detector.ac(), depth, lateral, true, true, stationary);

      Serial.printf("# PROCESS: x=%d y=%d → dist=%.1f depth=%.1f lat=%.1f ac=%.2f\n", 
                    t.x, t.y, dist, depth, lateral, detector.ac());

      locked_index = t.index;
      dbg_samples++;
      continue;
    }

    // Нет цели
    if (locked_index != -1) {
      detector.reset();
      locked_index = -1;
    }
    bridge::sendSample(0.0f, 0.0f, 0.0f, 0.0f, false, false, false);
  }
}

void setup() {
  bridge::begin(PC_BAUD);
  radarSerial.begin(LD2450_BAUD, SERIAL_8N1, LD2450_RX_PIN, LD2450_TX_PIN);
  Serial.println("ESP32 LD2450 Breath Detector v2 - Depth Fix");
}

void loop() {
  pollRadar(millis());

  static uint32_t last_summary = 0;
  uint32_t now = millis();
  if (now - last_summary >= SUMMARY_MS) {
    last_summary = now;
    bridge::sendSummary(detector.update(now), false, false, false); // упрощённо
  }
}