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

// NMEA 2000 device identity.
// TODO: kDeviceFunction/kDeviceClass need to be set to the correct
// values for a compass/heading sensor per the NMEA 2000 device class and
// function assignment list before this is used on the bus — the values
// below are placeholders inherited from the parent gateway firmware and
// are almost certainly wrong for this product.
constexpr uint16_t kManufacturerCode = 2046;  // Hat Labs
constexpr uint8_t kDeviceFunction = 0;        // TODO: verify (Compass?)
constexpr uint8_t kDeviceClass = 0;           // TODO: verify (Navigation?)

#endif  // HALSER_SRC_HALSER_CONST_H_
