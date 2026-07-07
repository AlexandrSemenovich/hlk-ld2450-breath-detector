// ESP32 firmware for HLK-LD2450 — real-time breath detection on-board.
//
// Module layout:
//   ld2450_parser.h/.cpp  -> UART frame parsing (up to 3 targets)
//   breath_detector.h/.cpp-> distance -> bpm + quality (per-sample)
//   radar_bridge.h        -> low-latency UART stream to PC
//   main.cpp              -> wiring + loop
//
// Coordinates follow LD2450 protocol (signed int16, MSB = sign, unit mm).
// Each radar sample is processed immediately via Detector::push() so the
// detection runs at the radar's own frame rate (no fixed-rate mismatch).
//
// RANGING / ZONE POLICY
//   * Parsing runs ALWAYS (we must see frames to detect zone entry).
//   * Breath processing (detector + D/S stream) runs ONLY for a target that
//     is present AND inside the analyzed zone.
//   * Zone entry : a target appears in-zone -> it is locked and tracked.
//   * Zone exit  : the locked target leaves the zone (or vanishes) -> the
//     detector is RESET, so no stale data is reported and the next entry
//     starts from a clean state (no jump in the filtered distance).
//
// Target distance is the RADIAL value r = sqrt(lateral^2 + depth^2) (orientation
// independent; on this module the front depth is reported in Y, while X is
// the lateral offset from the centerline.
//
// Analyzed zone:
//   radial distance r in [ZONE_R_MIN, ZONE_R_MAX]   (in front of radar)
//   lateral offset   |y|       <= ZONE_SIDE_MAX      (not too far to the side)
//
// Stream protocol (one line per message):
//   D<dist_mm>,<ac_mm>    distance + breath signal, every in-zone frame
//   S<bpm>,<amp>,<qual>   summary, sent periodically (zeros when no target)
//   # dbg ...              diagnostic line (when DEBUG=1)

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
static constexpr uint32_t PC_BAUD       = 921600;

#ifndef DEBUG
#define DEBUG 1                       // set 0 to disable diagnostic output
#endif

// Target with |speed| above this (cm/s) is likely moving too much for
// breathing detection. We keep a relaxed threshold because the LD2450 speed
// field is noisy and can report large values even for a stationary target.
static constexpr int16_t  MOTION_SPEED_CMS = 60;
// Drop the parser buffer if no valid frame arrived for this long (desync guard).
static constexpr uint32_t FRAME_TIMEOUT_MS = 1000;
static constexpr uint32_t SUMMARY_MS        = 500;    // S<...> output period
static constexpr uint32_t ZONE_EXIT_HYST_MS = 100; // short hysteresis to avoid flicker
static constexpr float    STATIONARY_SPEED_THRESHOLD = 80.0f; // cm/s
static constexpr float    STATIONARY_VEL_MM_S = 100.0f;      // mm/s

// Physically valid radial range for the LD2450 (mm). Anything outside is
// treated as garbage / empty slot (e.g. r=52309 seen on this module).
static constexpr float    VALID_R_MIN = 100.0f;
static constexpr float    VALID_R_MAX = 6000.0f;      // module max range = 6 m

// Analyzed zone (mm). For breath monitoring with a person at 0.8–1.2 m:
//   radial distance r in [ZONE_R_MIN, ZONE_R_MAX]
//   lateral offset   |y|       <= ZONE_SIDE_MAX
static constexpr float    ZONE_R_MIN    = 800.0f;
static constexpr float    ZONE_R_MAX    = 1200.0f;
static constexpr float    ZONE_SIDE_MAX = 150.0f;

HardwareSerial radarSerial(2);

// ---------------------------------------------------------------------------
// Runtime components
// ---------------------------------------------------------------------------
static ld2450::Parser      parser;
static breath::Detector    detector({});   // default tuning
static int16_t             locked_index = -1;  // -1 = no in-zone target locked
static uint32_t            last_frame_ms = 0;   // last valid frame seen
static uint32_t            last_summary_ms = 0;
static uint32_t            outzone_since_ms = 0;
static bool                state_visible = false;
static bool                state_in_zone = false;
static bool                state_stationary = false;
static float               last_tracked_dist = 0.0f;
static uint32_t            last_tracked_time = 0;
static bool                last_tracked_dist_valid = false;

// Debug counters
static uint32_t            dbg_bytes = 0;
static uint32_t            dbg_frames = 0;
static uint32_t            dbg_samples = 0;      // D lines emitted (in-zone)
static uint32_t            dbg_outzone = 0;      // frames with no in-zone target
static uint32_t            dbg_last_report = 0;
static ld2450::Frame       dbg_last_frame{};      // last valid frame for dump

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static float radial(const ld2450::Target& t) {
  const float lateral = static_cast<float>(t.x);
  const float depth = static_cast<float>(t.y);
  return sqrtf(lateral * lateral + depth * depth);
}

// A slot holds a real target only if its radial distance is physically
// plausible for the LD2450 (rejects empty slots and garbage like r=52309).
static bool isPresent(const ld2450::Target& t) {
  const float r = radial(t);
  return (r >= VALID_R_MIN && r <= VALID_R_MAX);
}

// True if the target is inside the analyzed zone.
static bool inZone(const ld2450::Target& t) {
  const float r = radial(t);
  const float lateral = static_cast<float>(t.x);
  return (r >= ZONE_R_MIN && r <= ZONE_R_MAX && fabsf(lateral) <= ZONE_SIDE_MAX);
}

// Pick the breathing target inside the zone.
//   * If a target is already locked and still present+in-zone, keep it
//     (hysteresis: avoids jumping between people / slots).
//   * Otherwise choose the nearest nearly-stationary in-zone target.
// Returns -1 when NO target is both present and inside the zone.
static int16_t pickTarget(const ld2450::Frame& f, int16_t locked) {
  if (locked >= 0) {
    for (uint8_t i = 0; i < f.count; i++) {
      if (f.targets[i].index != static_cast<uint8_t>(locked)) continue;
      const ld2450::Target& t = f.targets[i];
      if (!isPresent(t) || !inZone(t)) return -1;   // left zone / vanished
      return static_cast<int16_t>(i);
    }
    return -1;  // locked target disappeared from the frame -> force reset
  }

  int16_t best = -1;
  float   best_score = 1e12f;
  for (uint8_t i = 0; i < f.count; i++) {
    const ld2450::Target& t = f.targets[i];
    if (!isPresent(t) || !inZone(t)) continue;       // outside zone -> ignore
    const float r = radial(t);
    const bool stationary = (t.speed > -MOTION_SPEED_CMS) &&
                            (t.speed <  MOTION_SPEED_CMS);
    const float score = stationary ? r : (r + 1e6f);
    if (score < best_score) { best_score = score; best = static_cast<int16_t>(i); }
  }
  return best;
}

static void printHex(const uint8_t* p, uint16_t n) {
  for (uint16_t i = 0; i < n; i++) {
    if (i && (i % 16) == 0) Serial.println();
    Serial.printf("%02X ", p[i]);
  }
  Serial.println();
}

static void reportDebug(uint32_t now) {
  uint8_t raw[64];
  const uint16_t n = parser.lastRaw(raw, sizeof(raw));
  const uint8_t hdr_n_targets = (n > 4) ? raw[4] : 0;

  Serial.print("# state bytes=");
  Serial.print(dbg_bytes);
  Serial.print(" frames=");
  Serial.print(dbg_frames);
  Serial.print(" samples=");
  Serial.print(dbg_samples);
  Serial.print(" outzone=");
  Serial.print(dbg_outzone);
  Serial.print(" parserBuf=");
  Serial.print(parser.size());
  Serial.print(" age=");
  Serial.print(now - last_frame_ms);
  Serial.print("ms lock=");
  Serial.print(locked_index);
  Serial.print(" hdrNTargets=");
  Serial.print(hdr_n_targets);
  Serial.print(" parsedCount=");
  Serial.print(dbg_last_frame.count);

  if (dbg_last_frame.count == 0) {
    Serial.println(" | no targets parsed");
  } else {
    Serial.println(" | targets:");
    for (uint8_t i = 0; i < dbg_last_frame.count; i++) {
      const ld2450::Target& t = dbg_last_frame.targets[i];
      const bool present = isPresent(t);
      const bool zone = inZone(t);
      Serial.print("   slot");
      Serial.print(i);
      Serial.print(" -> r=");
      Serial.print(static_cast<int>(radial(t)));
      Serial.print("mm lat=");
      Serial.print(t.x);
      Serial.print("mm depth=");
      Serial.print(t.y);
      Serial.print("mm v=");
      Serial.print(t.speed);
      Serial.print("cm/s present=");
      Serial.print(present ? "Y" : "N");
      Serial.print(" zone=");
      Serial.println(zone ? "Y" : "N");
    }
  }

  bool visible_outside_zone = false;
  for (uint8_t i = 0; i < dbg_last_frame.count; i++) {
    const ld2450::Target& t = dbg_last_frame.targets[i];
    if (isPresent(t) && !inZone(t)) {
      visible_outside_zone = true;
      break;
    }
  }

  if (locked_index >= 0) {
    Serial.println("# action: tracking locked target inside zone");
  } else if (visible_outside_zone) {
    Serial.println("# status: target visible but outside analyzed zone");
  } else {
    Serial.println("# action: waiting for target to enter zone");
  }

  if (n) {
    Serial.print("# raw frame (");
    Serial.print(n);
    Serial.println(" bytes):");
    printHex(raw, n);
  }
}

static void pollRadar(uint32_t now) {
  // Recover from a stuck/desynced buffer.
  if (parser.size() > 0 && (now - last_frame_ms) > FRAME_TIMEOUT_MS) {
    parser.reset();
  }

  while (radarSerial.available()) {
    parser.feed(static_cast<uint8_t>(radarSerial.read()));
    dbg_bytes++;
  }

  ld2450::Frame f;
  while (parser.drain(f)) {
    if (!f.valid) continue;
    last_frame_ms = now;
    dbg_frames++;
    dbg_last_frame = f;

    bool visible = false;
    bool in_zone = false;
    bool stationary = false;

    for (uint8_t i = 0; i < f.count; i++) {
      const ld2450::Target& t = f.targets[i];
      if (!isPresent(t)) continue;
      visible = true;
      if (inZone(t)) in_zone = true;
    }

    const int16_t sel = pickTarget(f, locked_index);
    float visible_lateral = 0.0f;
    float visible_depth = 0.0f;
    int16_t visible_sel = -1;
    for (uint8_t i = 0; i < f.count; i++) {
      const ld2450::Target& t = f.targets[i];
      if (!isPresent(t)) continue;
      visible_sel = static_cast<int16_t>(i);
      break;
    }
    if (visible_sel >= 0) {
      const ld2450::Target& vt = f.targets[visible_sel];
      visible_lateral = static_cast<float>(vt.x);
      visible_depth = static_cast<float>(vt.y);
    }

    if (sel >= 0) {
      const ld2450::Target& t = f.targets[sel];
      in_zone = true;
      const float lateral = static_cast<float>(t.x);
      const float depth = static_cast<float>(t.y);
      const float dist = sqrtf(lateral * lateral + depth * depth);
      const float speed_val = fabsf(static_cast<float>(t.speed));
      const bool speed_stationary = (speed_val <= STATIONARY_SPEED_THRESHOLD);
      const float dt_s = (last_tracked_dist_valid && last_tracked_time > 0)
          ? (static_cast<float>(now - last_tracked_time) * 0.001f)
          : 0.0f;
      const bool dist_stationary = (dt_s > 0.0f)
          ? (fabsf(dist - last_tracked_dist) / dt_s < STATIONARY_VEL_MM_S)
          : false;
      stationary = speed_stationary || dist_stationary;
      last_tracked_dist = dist;
      last_tracked_time = now;
      last_tracked_dist_valid = true;
      visible_lateral = static_cast<float>(t.x);
      visible_depth = static_cast<float>(t.y);
    }

    state_visible = visible;
    state_in_zone = in_zone;
    state_stationary = stationary;

    if (sel < 0) {
      if (locked_index != -1) {
        Serial.println("# event: exit zone -> reset detector");
        detector.reset();
        locked_index = -1;
        last_tracked_dist_valid = false;
        outzone_since_ms = 0;
        dbg_outzone++;
        bridge::sendSample(0.0f, 0.0f, visible_depth, visible_lateral, visible, false, false);
        continue;
      }

      outzone_since_ms = 0;
      dbg_outzone++;
      bridge::sendSample(0.0f, 0.0f, visible_depth, visible_lateral, visible, false, false);
      continue;
    }

    // Target is present and inside the zone -> process it.
    if (locked_index < 0) {
      Serial.println("# event: enter zone -> start tracking");
    }
    outzone_since_ms = 0;
    locked_index = f.targets[sel].index;

    const float lateral = static_cast<float>(f.targets[sel].x);
    const float depth = static_cast<float>(f.targets[sel].y);
    const float dist = sqrtf(lateral * lateral + depth * depth);
    detector.push(dist, now);
    // The LD2450 reports X as lateral offset and Y as depth toward the scene.
    bridge::sendSample(dist, detector.ac(), depth, lateral, true, true, stationary);
    Serial.print("# event: process sample dist=");
    Serial.print(dist, 1);
    Serial.print("mm ac=");
    Serial.println(detector.ac(), 2);
    dbg_samples++;
  }
}

// ---------------------------------------------------------------------------
// Arduino lifecycle
// ---------------------------------------------------------------------------
void setup() {
  bridge::begin(PC_BAUD);
  radarSerial.begin(LD2450_BAUD, SERIAL_8N1, LD2450_RX_PIN, LD2450_TX_PIN);
  last_summary_ms = millis();
  last_frame_ms = millis();
  dbg_last_report = millis();
}

void loop() {
  const uint32_t now = millis();

  pollRadar(now);

  // Periodic summary. When no in-zone target is tracked, the detector was
  // reset, so this reports 0.0,0.0,0 (clean idle state).
  if (now - last_summary_ms >= SUMMARY_MS) {
    last_summary_ms = now;
    bridge::sendSummary(detector.update(now), state_visible, state_in_zone, state_stationary);
  }

#if DEBUG
  if (now - dbg_last_report >= 1000) {
    dbg_last_report = now;
    reportDebug(now);
  }
#endif
}
