#include "rate_of_turn.h"

#include <cmath>

namespace halser {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegreesToRadians = kPi / 180.0f;

// Shortest signed angular difference from `from` to `to`, both in
// degrees [0, 360), result in (-180, 180]. Positive = `to` is clockwise
// (increasing heading) from `from`.
float AngularDifferenceDegrees(float from, float to) {
  float diff = fmodf(to - from + 540.0f, 360.0f) - 180.0f;
  return diff;
}

}  // namespace

void RateOfTurnEstimator::AddSample(float heading_degrees,
                                     unsigned long timestamp_ms) {
  samples_[next_] = Sample{heading_degrees, timestamp_ms};
  next_ = (next_ + 1) % kMaxSamples;
  if (count_ < kMaxSamples) count_++;
}

bool RateOfTurnEstimator::GetRateOfTurn(float* rate_of_turn_rad_per_s) const {
  if (count_ < 2 || rate_of_turn_rad_per_s == nullptr) return false;

  const Sample& newest = samples_[(next_ + kMaxSamples - 1) % kMaxSamples];

  // Clamp rather than let this underflow when the device has been up for
  // less time than window_ms_ (unsigned arithmetic would otherwise wrap
  // to a huge value and reject every sample as "too old").
  unsigned long window_start_ms =
      (newest.timestamp_ms >= window_ms_) ? (newest.timestamp_ms - window_ms_)
                                            : 0;

  // Chronologically oldest currently-buffered sample: index 0 if the
  // ring buffer hasn't wrapped yet, otherwise `next_` (the slot about to
  // be overwritten holds the oldest surviving sample).
  size_t oldest_index = (count_ < kMaxSamples) ? 0 : next_;

  // Find the offset (within the chronological walk starting at
  // oldest_index) of the first sample that's within the window. Every
  // sample from there through `newest` participates in the regression
  // below; samples older than the window are simply skipped, same as
  // before.
  size_t start_offset = count_;  // sentinel: "none found"
  for (size_t i = 0; i < count_; i++) {
    size_t idx = (oldest_index + i) % kMaxSamples;
    if (samples_[idx].timestamp_ms >= window_start_ms) {
      start_offset = i;
      break;
    }
  }
  if (start_offset == count_) return false;  // defensive; newest always qualifies

  size_t first_idx = (oldest_index + start_offset) % kMaxSamples;
  unsigned long elapsed_ms = newest.timestamp_ms - samples_[first_idx].timestamp_ms;
  if (elapsed_ms < min_span_ms_) return false;

  // Windowed least-squares slope over every sample in the window, not
  // just the two endpoints -- a plain two-point finite difference is
  // known to amplify per-sample noise, since it throws away every
  // sample in between. Fitting a line and taking its slope uses all the
  // buffered data and is the standard technique for differentiating
  // noisy sampled signals. Heading is unwrapped relative to the first
  // included sample (each step advances by the shortest signed angular
  // delta) so the fit isn't corrupted by the 0/360 wrap boundary.
  double sum_t = 0.0, sum_h = 0.0, sum_tt = 0.0, sum_th = 0.0;
  size_t n = 0;
  float unwrapped_heading = samples_[first_idx].heading_degrees;
  float prev_heading = unwrapped_heading;
  unsigned long base_ms = samples_[first_idx].timestamp_ms;

  for (size_t i = start_offset; i < count_; i++) {
    size_t idx = (oldest_index + i) % kMaxSamples;
    const Sample& sample = samples_[idx];
    if (i > start_offset) {
      unwrapped_heading += AngularDifferenceDegrees(prev_heading, sample.heading_degrees);
      prev_heading = sample.heading_degrees;
    }
    double t = static_cast<double>(sample.timestamp_ms - base_ms);
    double h = static_cast<double>(unwrapped_heading);
    sum_t += t;
    sum_h += h;
    sum_tt += t * t;
    sum_th += t * h;
    n++;
  }

  double denominator = static_cast<double>(n) * sum_tt - sum_t * sum_t;
  if (denominator == 0.0) return false;  // all samples at the same timestamp

  double slope_degrees_per_ms =
      (static_cast<double>(n) * sum_th - sum_t * sum_h) / denominator;

  *rate_of_turn_rad_per_s =
      static_cast<float>(slope_degrees_per_ms * 1000.0) * kDegreesToRadians;
  return true;
}

}  // namespace halser
