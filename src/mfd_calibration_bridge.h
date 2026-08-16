#ifndef HALSER_SRC_MFD_CALIBRATION_BRIDGE_H_
#define HALSER_SRC_MFD_CALIBRATION_BRIDGE_H_

#include <NMEA2000.h>

#include "hwt3100_calibration_commands.h"

namespace halser {

// Lets a compatible MFD start/stop the HWT3100's on-module calibration
// over the N2K bus (SPEC.md §8.2, §10), the same way
// htool/ESP32_Precision-9_compass_CMPS14 does it for its CMPS14 sensor.
//
// PGN 130850/130851 is an undocumented, reverse-engineered proprietary
// Navico/Simnet message, not a published NMEA 2000 spec — see
// docs/plans/mfd-calibration.md for the full caveat and field layout.
// This has NOT been verified against real MFD hardware or a real N2K
// bus; it's a best-effort port of a working reference implementation's
// approach, not a guarantee.
//
// Dispatches to the *existing* CalibrationCommandHandler
// (hwt3100_calibration_commands.h) rather than reaching
// HWT3100SerialIO::SendCommand() directly — this MFD-triggered path
// reuses the same allowlisted write chokepoint the web UI calibration
// triggers already go through (ARCHITECTURE.md §2.2, §6), not a second
// one.
class MfdCalibrationBridge : public tNMEA2000::tMsgHandler {
 public:
  // "Command4 == 18" is htool's reverse-engineered marker for "this is
  // a calibration command" within PGN 130850's payload.
  static constexpr unsigned char kCalibrationCommandClass = 18;

  // Compared against the incoming message's first command byte. Copied
  // verbatim from the reference implementation's DEVICE_ID=24 — its
  // exact meaning (an N2K device instance? something else?) is
  // undocumented even there; see SPEC.md §10 for the decision to copy
  // it as-is rather than guess at a different value.
  static constexpr unsigned char kMfdDeviceId = 24;

  MfdCalibrationBridge(tNMEA2000* nmea2000,
                       CalibrationCommandHandler* calibration_commands)
      : tNMEA2000::tMsgHandler(130850L, nmea2000),
        nmea2000_(nmea2000),
        calibration_commands_(calibration_commands) {}

  void HandleMsg(const tN2kMsg& msg) override;

 private:
  void SendAck(unsigned char stop_start);

  tNMEA2000* nmea2000_;
  CalibrationCommandHandler* calibration_commands_;
};

}  // namespace halser

#endif  // HALSER_SRC_MFD_CALIBRATION_BRIDGE_H_
