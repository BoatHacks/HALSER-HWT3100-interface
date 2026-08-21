# AGENTS.md

## Project Overview

HALSER-HWT3100-interface — ESP32-C3 firmware that reads magnetic heading
from a WitMotion HWT3100-TTL/232 fluxgate compass over serial, and
republishes it as an NMEA 2000 message and a SignalK delta. Read
[SPEC.md](SPEC.md) and [ARCHITECTURE.md](ARCHITECTURE.md) before making
changes — this document is a quick-reference summary of those, not a
replacement for them.

**Hardware limitation, not a bug**: the HWT3100-TTL/232 is a compass, not
an IMU — it has no accelerometer/gyroscope and cannot produce pitch/roll.
Don't add attitude/PGN 127257 support expecting it to work; see SPEC.md
§1.2 and §9.3.

**Hard safety constraint**: never implement `AT+MODE` in any form. See
SPEC.md §1.2/§2 and ARCHITECTURE.md §6 for why — a real unit has been
bricked by this command, and this firmware has no functional need for
Modbus mode (ASCII mode already provides everything required).

## Build Commands

```bash
# Build firmware
pio run

# Upload to connected board
pio run -t upload

# Monitor serial output
pio device monitor
```

## Architecture

See ARCHITECTURE.md for the full component breakdown. Summary:

### Data Flow

```
UART1 (9600 baud default, GPIO 3 RX)
  → HWT3100 ASCII line parser ("Magx:<n>,Magy:<n>,Magz:<n>,Yaw:<n.n>\r\n")
  → HeadingReading (heading, magX/Y/Z, timestamp)
  → Calibration offset applied
  → N2K sender (PGN 127250 only) + SignalK delta sender, independently
    toggleable
```

Calibration commands (`AT+CALI=0/1/2`) are sent through a separate,
allowlisted write path — see ARCHITECTURE.md §2.1, §2.2, §6.

### Source Layout

- `src/main.cpp` — entry point
- `src/halser_const.h` — pin assignments and constants
- `src/gateway.h/.cpp` — SensESP application wiring (not yet implemented
  — see IMPLEMENTATION_CHECKLIST.md)
- Planned (see ARCHITECTURE.md §7 for the full file structure):
  `hwt3100_serial.h/.cpp`, `hwt3100_calibration_commands.h/.cpp`,
  `calibration_offset.h`, `n2k_senders.h`, `serial_terminal.h/.cpp`

### System Health Reporting

`gateway.cpp` calls `SensESPAppBuilder::enable_system_info_sensors()`,
which publishes SensESP's built-in system-health sensors to SignalK under
`sensors.halser-hwt3100.*`: `systemHz` (event loop rate), `freeMemory`,
`uptime`, `ipAddress`, and `wifiSignalLevel`.

### Hardware Pin Assignments

| Pin | Function |
|-----|----------|
| GPIO 2 | UART1 TX (to HWT3100, via HALSER's UART terminal block, jumper on "U") |
| GPIO 3 | UART1 RX (from HWT3100) |
| GPIO 4 | CAN TX |
| GPIO 5 | CAN RX |
| GPIO 8 | RGB LED (SK6805) |
| GPIO 9 | Button |

## Dependencies

- SensESP 3.2.0 — IoT framework (WiFi, web UI, Signal K)
- NMEA2000-library — NMEA 2000 message handling
- NMEA2000_twai — ESP32 TWAI (CAN) driver
- Adafruit NeoPixel — RGB LED control
