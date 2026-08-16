#ifndef HALSER_SRC_HWT3100_CALIBRATION_COMMANDS_H_
#define HALSER_SRC_HWT3100_CALIBRATION_COMMANDS_H_

#include "hwt3100_serial.h"
#include "hwt3100_types.h"

namespace halser {

// The only component that calls HWT3100SerialIO::SendCommand() to
// actually decide when a calibration command fires (SPEC.md §8.2,
// ARCHITECTURE.md §2.2, §6). Named actions only, one method per
// HWT3100Command value — there is no method here, or anywhere in this
// firmware, that forwards arbitrary/user-supplied text to the module.
//
// Small enough to stay header-only rather than split into a .cpp: each
// method is a single call straight through to SendCommand().
class CalibrationCommandHandler {
 public:
  explicit CalibrationCommandHandler(HWT3100SerialIO* serial_io)
      : serial_io_(serial_io) {}

  void StartCalibration() {
    serial_io_->SendCommand(HWT3100Command::kStartCalibration);
  }

  void EndCalibration() {
    serial_io_->SendCommand(HWT3100Command::kEndCalibration);
  }

  void ClearCalibration() {
    serial_io_->SendCommand(HWT3100Command::kClearCalibration);
  }

 private:
  HWT3100SerialIO* serial_io_;
};

}  // namespace halser

#endif  // HALSER_SRC_HWT3100_CALIBRATION_COMMANDS_H_
