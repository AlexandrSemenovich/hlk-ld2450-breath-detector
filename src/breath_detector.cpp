#include "breath_detector.h"
#include <cmath>

namespace breath {

void Detector::push(float distance_mm, uint32_t now_ms) {
  if (!primed_) {
    lp_ = distance_mm;     // initialize filter on first sample
    trend_ = distance_mm;
    last_ms_ = now_ms;
    primed_ = true;
    result_.distance = lp_;
    return;
  }

  // Real dt between samples (clamp to avoid spikes on long gaps).
  uint32_t dms = now_ms - last_ms_;
  if (dms > 500) dms = 500;
  last_ms_ = now_ms;
  const float dt = static_cast<float>(dms);

  // Low-pass the raw distance.
  lp_ = cfg_.lp_alpha * distance_mm + (1.0f - cfg_.lp_alpha) * lp_;

  // Slow high-pass: trend follows lp with tau; ac is the residual.
  const float a = 1.0f - expf(-dt / static_cast<float>(cfg_.trend_tau_ms));
  trend_ += a * (lp_ - trend_);
  ac_ = lp_ - trend_;

  // Ring-buffer history.
  const uint16_t hsize = historySize();
  hist_[hist_idx_] = ac_;
  hist_idx_ = (hist_idx_ + 1) % hsize;
  if (hist_fill_ < hsize) hist_fill_++;

  // Zero-crossing (rising edge through zero) -> period -> bpm.
  const bool is_pos = ac_ > 0.0f;
  if (is_pos && !was_pos_) {
    if (last_zc_ms_ != 0) {
      const uint32_t period = now_ms - last_zc_ms_;
      if (period >= cfg_.zc_min_period && period <= cfg_.zc_max_period) {
        const float bpm = 60000.0f / static_cast<float>(period);
        bpm_ = (bpm_ == 0.0f) ? bpm : (bpm_ * 0.7f + bpm * 0.3f);
      }
    }
    last_zc_ms_ = now_ms;
  }
  was_pos_ = is_pos;

  // Decay rate if no recent crossing.
  if (now_ms - last_zc_ms_ > cfg_.zc_max_period) bpm_ = 0.0f;

  // Quality + amplitude from AC RMS.
  if (hist_fill_ >= hsize / 4) {
    float mean = 0.0f;
    for (uint16_t i = 0; i < hist_fill_; i++) mean += hist_[i];
    mean /= hist_fill_;
    float var = 0.0f;
    for (uint16_t i = 0; i < hist_fill_; i++) {
      const float d = hist_[i] - mean;
      var += d * d;
    }
    var /= hist_fill_;
    const float rms = sqrtf(var);
    result_.amplitude = rms * 2.0f;   // peak-to-peak approx
    float qn = (rms - 0.3f) / 4.0f;
    if (qn < 0.0f) qn = 0.0f;
    if (qn > 1.0f) qn = 1.0f;
    result_.quality = qn * 100.0f;
  }

  result_.bpm = bpm_;
  result_.distance = lp_;
}

uint16_t Detector::getWave(float* out, uint16_t n) const {
  const uint16_t hsize = historySize();
  const uint16_t avail = (hist_fill_ < hsize) ? hist_fill_ : hsize;
  if (n > avail) n = avail;
  // Oldest sample sits at hist_idx_ (ring start); copy n most-recent.
  const uint16_t start = (hist_idx_ + hsize - n) % hsize;
  for (uint16_t i = 0; i < n; i++) {
    out[i] = hist_[(start + i) % hsize];
  }
  return n;
}

void Detector::reset() {
  lp_ = trend_ = ac_ = 0.0f;
  primed_ = false;
  last_ms_ = 0;
  hist_idx_ = hist_fill_ = 0;
  was_pos_ = false;
  last_zc_ms_ = 0;
  bpm_ = 0.0f;
  result_ = Result{};
}

}  // namespace breath
