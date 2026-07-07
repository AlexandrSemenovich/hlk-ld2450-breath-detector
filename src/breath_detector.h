#pragma once
// Real-time breath detector.
// Turns a stream of radial distances (mm) into a breath rate estimate (bpm)
// plus signal amplitude and a 0..100 quality score.
//
// Pipeline per sample (call push() for every radar sample):
//   lp   = EMA low-pass (remove HF jitter)
//   ac   = lp - trend ; trend = slow EMA high-pass (remove body drift)
//   bpm  = zero-crossing period of ac, smoothed
// update() just returns the latest computed Result.

#include <cstdint>

namespace breath {

struct Config {
  float    lp_alpha;       // EMA smoothing for lp (smaller = smoother)
  uint32_t trend_tau_ms;   // high-pass time constant (drift removal)
  uint32_t zc_min_period;  // ms -> upper bpm bound (e.g. 1200 = 50 bpm)
  uint32_t zc_max_period;  // ms -> lower bpm bound  (e.g. 20000 = 3 bpm)
  uint16_t history_s;      // seconds of AC history for quality estimate

  Config()
      : lp_alpha(0.15f),
        trend_tau_ms(6000),
        zc_min_period(1000),   // 60 bpm upper bound
        zc_max_period(15000),  // 4 bpm lower bound
        history_s(20) {}
};

struct Result {
  float bpm      = 0.0f;   // breaths per minute (0 = none detected)
  float amplitude = 0.0f;  // peak-to-peak estimate (mm)
  float quality  = 0.0f;   // 0..100
  float distance = 0.0f;   // current filtered distance (mm)
};

class Detector {
 public:
  explicit Detector(const Config& cfg) : cfg_(cfg) {}

  const Config& cfg() const { return cfg_; }
  float debugLp() const { return lp_; }
  bool debugPrimed() const { return primed_; }
  float ac() const { return ac_; }          // detrended breath signal (mm)
  uint16_t fill() const { return hist_fill_; }

  // Feed one raw radial distance (mm) measured at `now_ms`.
  // Performs all filtering/detection for this sample.
  void push(float distance_mm, uint32_t now_ms);

  // Return the latest computed result (no heavy work).
  const Result& update(uint32_t /*now_ms*/) const { return result_; }

  void reset();

 private:
  uint16_t historySize() const { return cfg_.history_s * 10u; }  // ~10 Hz (LD2450 datasheet)

  const Config cfg_;

  float    lp_      = 0.0f;
  float    trend_   = 0.0f;
  float    ac_      = 0.0f;
  bool     primed_  = false;
  uint32_t last_ms_ = 0;

  Result   result_;

  // AC history ring buffer.
  static constexpr uint16_t HIST_MAX = 400;  // 20 s @ 20 Hz
  float    hist_[HIST_MAX];
  uint16_t hist_idx_ = 0;
  uint16_t hist_fill_ = 0;

  bool     was_pos_  = false;
  uint32_t last_zc_ms_ = 0;
  float    bpm_      = 0.0f;
};

}  // namespace breath
