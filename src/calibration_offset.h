#ifndef HALSER_SRC_CALIBRATION_OFFSET_H_
#define HALSER_SRC_CALIBRATION_OFFSET_H_

#include <cmath>

#include "hwt3100_types.h"

namespace halser {

// Applies the configured heading calibration offset (SPEC.md §2, §9) to
// a HeadingReading, wrapping the result into [0, 360). Pure function —
// magnetic field fields pass through unchanged; they're diagnostic-only
// (SPEC.md §5.2) and don't need offset correction for that purpose.
inline HeadingReading ApplyCalibrationOffset(const HeadingReading& reading,
                                              float heading_offset_degrees) {
  HeadingReading corrected = reading;
  float heading = fmodf(reading.heading + heading_offset_degrees, 360.0f);
  if (heading < 0.0f) heading += 360.0f;
  corrected.heading = heading;
  return corrected;
}

}  // namespace halser

#endif  // HALSER_SRC_CALIBRATION_OFFSET_H_
