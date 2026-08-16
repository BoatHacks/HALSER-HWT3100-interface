#ifndef HALSER_SRC_HWT3100_TYPES_H_
#define HALSER_SRC_HWT3100_TYPES_H_

#include <cstddef>
#include <cstdint>

// See ARCHITECTURE.md §3. No pitch/roll fields: the HWT3100-TTL/232 is a
// compass, not an IMU, and cannot produce them (SPEC.md §1.2, §9.3).
struct HeadingReading {
  float heading = 0.0f;   // degrees, 0-360, magnetic, offset-corrected
  int32_t mag_x = 0;      // raw magnetic field X, diagnostic use only
  int32_t mag_y = 0;      // raw magnetic field Y, diagnostic use only
  int32_t mag_z = 0;      // raw magnetic field Z, diagnostic use only
  unsigned long timestamp = 0;  // millis() of last successful sensor read
};

// The complete, deliberately small set of commands this firmware will
// ever send to the HWT3100. There is no value here for AT+MODE — see
// SPEC.md §1.2/§2 and ARCHITECTURE.md §6 for why that's permanent.
enum class HWT3100Command {
  kStartCalibration,   // AT+CALI=1
  kEndCalibration,     // AT+CALI=0
  kClearCalibration,   // AT+CALI=2
};

// A raw line tapped from the HWT3100's serial output, for the live
// serial terminal (SPEC.md §8.1). Fixed-size POD rather than
// Arduino::String so it can cross the FreeRTOS task boundary via
// TaskQueueProducer without heap allocation (see SafeQueue's own
// documentation on why that matters for cross-task data).
struct HWT3100RawLine {
  static constexpr size_t kMaxLength = 128;
  char text[kMaxLength] = {0};
};

#endif  // HALSER_SRC_HWT3100_TYPES_H_
