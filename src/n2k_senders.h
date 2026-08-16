#ifndef HALSER_SRC_N2K_SENDERS_H_
#define HALSER_SRC_N2K_SENDERS_H_

#include <N2kMessages.h>
#include <NMEA2000.h>

namespace halser {

/// Tracks a value with automatic expiry for N2K message building.
/// When the value hasn't been updated within max_age_ms, to_n2k() returns
/// N2kDoubleNA instead of a stale value. Adapted from the parent
/// HALSER-default-firmware's n2k_senders.h — SPEC.md §6 requires exactly
/// this "stale -> N2K not-available value" behavior, which this class
/// already provides.
///
/// Thread safety: all update()/is_valid()/to_n2k() calls must occur on
/// the main SensESP event loop. HWT3100SerialIO marshals parsed readings
/// back to the main loop via TaskQueueProducer, so this holds in the
/// current architecture (see ARCHITECTURE.md §2.1, §2.6).
template <typename T>
class ExpiringValue {
 public:
  explicit ExpiringValue(unsigned long max_age_ms) : max_age_(max_age_ms) {}

  void update(const T& value) {
    value_ = value;
    last_update_ = millis();
  }

  bool is_valid() const {
    return last_update_ > 0 && (millis() - last_update_) < max_age_;
  }

  const T& value() const { return value_; }

  double to_n2k() const {
    return is_valid() ? static_cast<double>(value_) : N2kDoubleNA;
  }

 private:
  T value_{};
  unsigned long last_update_ = 0;
  unsigned long max_age_;
};

/// PGN 127250 — Vessel Heading (100ms). The only N2K PGN this firmware
/// implements — see SPEC.md §5.1, §9.3 for why there is deliberately no
/// PGN 127257 (Attitude) sender: the HWT3100 can't supply pitch/roll.
///
/// Unlike the parent firmware's version of this class, send() is
/// unconditional (no has_data() gate) — SPEC.md §6/§10 decided this
/// firmware transmits N2K "not available" values when stale rather than
/// omitting the PGN, and that decision applies equally to "never
/// received a reading yet" (also "not available").
class N2kHeadingSender {
 public:
  explicit N2kHeadingSender(tNMEA2000* nmea2000, unsigned long expiry = 5000)
      : nmea2000_(nmea2000), heading_(expiry) {}

  void send() {
    tN2kMsg msg;
    SetN2kPGN127250(msg, 0xff, heading_.to_n2k(), N2kDoubleNA, N2kDoubleNA,
                     N2khr_magnetic);
    nmea2000_->SendMsg(msg);
  }

  ExpiringValue<float> heading_;

 private:
  tNMEA2000* nmea2000_;
};

/// PGN 127251 — Rate of Turn (100ms). Unlike N2kHeadingSender, the value
/// fed to this sender is computed, not sensed — see RateOfTurnEstimator
/// (rate_of_turn.h) and SPEC.md §1.3/§5.1/§10 for why. "Not available"
/// applies identically whether the HWT3100 has gone stale or the
/// estimator simply doesn't have enough heading history yet — both cases
/// are "no rate of turn value right now," and ExpiringValue treats them
/// the same way.
class N2kRateOfTurnSender {
 public:
  explicit N2kRateOfTurnSender(tNMEA2000* nmea2000, unsigned long expiry = 5000)
      : nmea2000_(nmea2000), rate_of_turn_(expiry) {}

  void send() {
    tN2kMsg msg;
    SetN2kPGN127251(msg, 0xff, rate_of_turn_.to_n2k());
    nmea2000_->SendMsg(msg);
  }

  ExpiringValue<float> rate_of_turn_;

 private:
  tNMEA2000* nmea2000_;
};

}  // namespace halser

#endif  // HALSER_SRC_N2K_SENDERS_H_
