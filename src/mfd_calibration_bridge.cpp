#include "mfd_calibration_bridge.h"

namespace halser {

void MfdCalibrationBridge::HandleMsg(const tN2kMsg& msg) {
  if (msg.PGN != 130850L) return;

  // Skip the 2-byte manufacturer-code/industry-code header standard to
  // proprietary PGNs, then read the 5 command bytes htool's reference
  // implementation reverse-engineered (docs/plans/mfd-calibration.md).
  int index = 2;
  unsigned char command1 = msg.GetByte(index);
  msg.GetByte(index);  // Command2 — read but unused, matching the reference
  msg.GetByte(index);  // Command3 — read but unused, matching the reference
  unsigned char command4 = msg.GetByte(index);
  unsigned char calibration_stop_start = msg.GetByte(index);

  if (command1 != kMfdDeviceId || command4 != kCalibrationCommandClass) return;

  if (calibration_stop_start == 0) {
    calibration_commands_->StartCalibration();
    SendAck(0);
  } else if (calibration_stop_start == 1) {
    calibration_commands_->EndCalibration();
    SendAck(1);
  }
}

void MfdCalibrationBridge::SendAck(unsigned char stop_start) {
  tN2kMsg msg;
  msg.Init(7, 130851L, 0xff, 0xff);
  msg.AddByte(0x41);
  msg.AddByte(0x9f);
  msg.AddByte(kMfdDeviceId);
  msg.AddByte(0xff);
  msg.AddByte(0xff);
  msg.AddByte(kCalibrationCommandClass);
  msg.AddByte(stop_start);
  msg.AddByte(0x00);
  msg.AddByte(0xff);
  msg.AddByte(0xff);
  msg.AddByte(0xff);
  msg.AddByte(0xff);
  nmea2000_->SendMsg(msg);
}

}  // namespace halser
