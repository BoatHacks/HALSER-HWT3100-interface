#ifndef HALSER_SRC_HALSER_CONST_H_
#define HALSER_SRC_HALSER_CONST_H_

#include <driver/gpio.h>

// ESP32-C3 pin assignments for HALSER board.
// See ARCHITECTURE.md §5 (Integration Points) for the HWT3100 wiring
// note: UART1 connects to HALSER's "UART" terminal block with the
// RX-select jumper set to "U".
constexpr gpio_num_t kUART1TxPin = GPIO_NUM_2;
constexpr gpio_num_t kUART1RxPin = GPIO_NUM_3;
constexpr gpio_num_t kCANTxPin = GPIO_NUM_4;
constexpr gpio_num_t kCANRxPin = GPIO_NUM_5;
constexpr int kRGBLEDPin = 8;
constexpr int kButtonPin = 9;

// HWT3100-TTL/232 default baud rate (ARCHITECTURE.md §5, SPEC.md §1.2).
constexpr int kHWT3100DefaultBaud = 9600;

// NMEA 2000 device identity — cloned from the B&G Precision-9 compass's
// identity (SPEC.md §1.2, §5.1, §10), via the reference implementation
// htool/ESP32_Precision-9_compass_CMPS14. Deliberately NOT cloned: the
// "unique number" passed to SetDeviceInformation() — see gateway.cpp,
// which derives it from this board's own MAC address instead (SPEC.md
// §10 explains why).
constexpr uint16_t kManufacturerCode = 275;   // as used by the reference project
constexpr uint8_t kDeviceFunction = 140;      // per the reference project's NMEA2000 class/function reference
constexpr uint8_t kDeviceClass = 60;          // "Sensor Communication Interface"

// Product info fields for tNMEA2000::SetProductInformation(), also
// cloned from the Precision-9 reference implementation.
constexpr const char* kProductModelSerialCode = "107018103";
constexpr uint16_t kProductCode = 13233;
constexpr const char* kProductModelId = "Precision-9 Compass";
constexpr const char* kProductSoftwareVersion = "2.9.4-3";
constexpr const char* kProductModelVersion = "2";

#endif  // HALSER_SRC_HALSER_CONST_H_
