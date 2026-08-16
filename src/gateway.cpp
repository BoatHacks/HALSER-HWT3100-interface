#include "gateway.h"

#include <Arduino.h>

// TODO: not yet implemented. See IMPLEMENTATION_CHECKLIST.md and
// docs/plans/ for the planned build-out (HWT3100 serial I/O, calibration,
// N2K sender, SignalK delta sender, serial terminal, calibration
// commands — see ARCHITECTURE.md §2 for the full component list).
void run_hwt3100_gateway() {
  Serial.begin(115200);
  while (true) {
    Serial.println("HALSER-HWT3100-interface: not yet implemented");
    delay(5000);
  }
}
