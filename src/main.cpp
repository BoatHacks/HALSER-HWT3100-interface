#include <Arduino.h>

#include "gateway.h"

void setup() {
  run_hwt3100_gateway();  // Does not return
}

void loop() {
  // Not reached — run_hwt3100_gateway() runs its own event loop.
}
