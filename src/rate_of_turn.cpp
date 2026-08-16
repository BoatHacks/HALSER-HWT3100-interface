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

  const Sample* oldest_in_window = &newest;
  for (size_t i = 0; i < count_; i++) {
    size_t idx = (oldest_index + i) % kMaxSamples;
    if (samples_[idx].timestamp_ms >= window_start_ms) {
      oldest_in_window = &samples_[idx];
      break;
    }
  }

  unsigned long elapsed_ms =
      newest.timestamp_ms - oldest_in_window->timestamp_ms;
  if (elapsed_ms < min_span_ms_) return false;

  float diff_degrees =
      AngularDifferenceDegrees(oldest_in_window->heading_degrees, newest.heading_degrees);
  float elapsed_s = static_cast<float>(elapsed_ms) / 1000.0f;

  *rate_of_turn_rad_per_s = (diff_degrees * kDegreesToRadians) / elapsed_s;
  return true;
}

}  // namespace halser
