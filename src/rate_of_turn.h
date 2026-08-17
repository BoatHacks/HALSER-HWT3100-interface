#ifndef HALSER_SRC_RATE_OF_TURN_H_
#define HALSER_SRC_RATE_OF_TURN_H_

#include <cstddef>

namespace halser {

// Computes rate of turn (SPEC.md §1.3, §3) from a sliding window of
// heading readings — the HWT3100 has no gyroscope, so this is derived,
// not sensed (SPEC.md §5.1, §10). Pure logic, no Arduino dependency, so
// it's unit-testable on the host (see
// docs/plans/precision9-rate-of-turn.md).
//
// Not thread-safe: like the rest of this firmware's data pipeline, all
// calls must happen on the main SensESP event loop (ARCHITECTURE.md
// §2.1, §2.6).
class RateOfTurnEstimator {
 public:
  // window_ms: how far back samples are kept. min_span_ms: the minimum
  // elapsed time between the oldest and newest sample in the window
  // before GetRateOfTurn() will return a value — below this, two
  // samples a few milliseconds apart would turn ordinary sensor noise
  // into wild rate swings. See SPEC.md §11 for why these are flagged as
  // needing real-hardware tuning.
  RateOfTurnEstimator(unsigned long window_ms, unsigned long min_span_ms)
      : window_ms_(window_ms), min_span_ms_(min_span_ms) {}

  // Adds a heading sample (degrees, 0-360) at the given timestamp
  // (millis()). Timestamps must be non-decreasing across calls.
  void AddSample(float heading_degrees, unsigned long timestamp_ms);

  // Fills *rate_of_turn_rad_per_s (radians/second, positive = turning to
  // starboard) and returns true if the window holds samples spanning at
  // least min_span_ms; returns false (leaving the output untouched)
  // otherwise — e.g. right after startup, or after a stale-data gap
  // clears out old samples.
  bool GetRateOfTurn(float* rate_of_turn_rad_per_s) const;

  // Runtime-mutable window/min-span (SPEC.md §7, §11) — previously
  // fixed at construction. Read fresh by GetRateOfTurn() on every
  // call, so a change takes effect on the very next call; no need to
  // rebuild the ring buffer or reset any state.
  void SetWindowMs(unsigned long window_ms) { window_ms_ = window_ms; }
  void SetMinSpanMs(unsigned long min_span_ms) { min_span_ms_ = min_span_ms; }

 private:
  // Bounds memory for a fixed-size ring buffer. At a plausible HWT3100
  // update rate (up to ~10 Hz, per its documented 0.1-100 Hz range but
  // realistically what this firmware's serial read loop can sustain), a
  // few-second window comfortably fits; under a much higher rate than
  // expected, old samples get overwritten a bit before window_ms_,
  // trading window duration for bounded memory rather than unbounded
  // growth.
  static constexpr size_t kMaxSamples = 64;

  struct Sample {
    float heading_degrees = 0.0f;
    unsigned long timestamp_ms = 0;
  };

  Sample samples_[kMaxSamples];
  size_t count_ = 0;
  size_t next_ = 0;
  unsigned long window_ms_;
  unsigned long min_span_ms_;
};

}  // namespace halser

#endif  // HALSER_SRC_RATE_OF_TURN_H_
