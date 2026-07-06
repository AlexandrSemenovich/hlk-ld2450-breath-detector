// ESP32 firmware for HLK-LD2450 — real-time breath detection on-board.
//
// Module layout:
//   ld2450_parser.h/.cpp  -> UART frame parsing (up to 3 targets)
//   breath_detector.h/.cpp-> distance -> bpm + quality (per-sample)
//   radar_bridge.h        -> CSV output to PC
//   main.cpp              -> wiring + loop
//
// Coordinates follow LD2450 protocol (signed 16-bit, bit15 = sign).
// Each radar sample is processed immediately via Detector::push() so the
// detection runs at the radar's own frame rate (no fixed-rate mismatch).

#include <Arduino.h>
#include <HardwareSerial.h>

#include "ld2450_parser.h"
#include "breath_detector.h"
#include "radar_bridge.h"

// ---------------------------------------------------------------------------
// Hardware config
// ---------------------------------------------------------------------------
static constexpr uint8_t  LD2450_RX_PIN = 32;
static constexpr uint8_t  LD2450_TX_PIN = 33;
static constexpr uint32_t LD2450_BAUD   = 256000;
static constexpr uint32_t PC_BAUD       = 115200;

static constexpr uint32_t TARGET_TIMEOUT_MS = 3000;   // re-acquire after this
static constexpr uint32_t STREAM_MS         = 100;    // CSV output period

HardwareSerial radarSerial(2);

// ---------------------------------------------------------------------------
// Runtime components
// ---------------------------------------------------------------------------
static ld2450::Parser      parser;
static breath::Detector    detector({});   // default tuning
static int16_t             locked_index = -1;
static uint32_t            last_seen_ms = 0;
static uint32_t            last_stream_ms = 0;

// Wave buffer for the ASCII scope (re-used each stream tick).
static constexpr uint16_t  WAVE_N = 40;
static float               wave_buf[WAVE_N];
static constexpr float     WAVE_SCALE_MM = 6.0f;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
// Choose the nearest real target. LD2450 fills empty slots with (0,0),
// so ignore targets with near-zero radial distance.
static int16_t pickNearest(const ld2450::Frame& f) {
  if (f.count == 0) return -1;
  int16_t best = -1;
  float best_r = 1e12f;
  for (uint8_t i = 0; i < f.count; i++) {
    const float x = static_cast<float>(f.targets[i].x);
    const float y = static_cast<float>(f.targets[i].y);
    const float r = sqrtf(x * x + y * y);
    if (r < 1.0f) continue;
    if (r < best_r) { best_r = r; best = static_cast<int16_t>(i); }
  }
  return best;
}

static void pollRadar(uint32_t now) {
  while (radarSerial.available()) {
    parser.feed(static_cast<uint8_t>(radarSerial.read()));
  }

  ld2450::Frame f;
  while (parser.drain(f)) {
    if (!f.valid) continue;

    const int16_t sel = pickNearest(f);
    if (sel < 0) continue;

    // (Re)acquire nearest target on first lock or timeout.
    if (locked_index < 0 || (now - last_seen_ms) > TARGET_TIMEOUT_MS) {
      locked_index = sel;
    }
    if (f.targets[sel].index == static_cast<uint8_t>(locked_index)) {
      last_seen_ms = now;
      const float x = static_cast<float>(f.targets[sel].x);
      const float y = static_cast<float>(f.targets[sel].y);
      detector.push(sqrtf(x * x + y * y), now);
    }
  }
}

// ---------------------------------------------------------------------------
// Arduino lifecycle
// ---------------------------------------------------------------------------
void setup() {
  bridge::begin(PC_BAUD);
  radarSerial.begin(LD2450_BAUD, SERIAL_8N1, LD2450_RX_PIN, LD2450_TX_PIN);
  last_stream_ms = millis();
}

void loop() {
  const uint32_t now = millis();

  pollRadar(now);

  // Stream the latest result + an ASCII breath waveform at a fixed cadence.
  if (now - last_stream_ms >= STREAM_MS) {
    last_stream_ms = now;
    bridge::sendResult(detector.update(now));
    const uint16_t n = detector.getWave(wave_buf, WAVE_N);
    bridge::sendWave(wave_buf, n, WAVE_SCALE_MM);
  }
}
