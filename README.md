# HALSER-HWT3100-interface

WitMotion HWT3100-TTL compass to NMEA 2000 / SignalK bridge firmware for the [HALSER](https://shop.hatlabs.fi/products/halser) ESP32-C3 serial interface board.

Reads magnetic heading from a HWT3100-TTL/232 fluxgate electronic compass
over UART, and republishes it as both an NMEA 2000 message and a SignalK
delta — either output independently enable/disable-able.

See [SPEC.md](SPEC.md) for requirements and [ARCHITECTURE.md](ARCHITECTURE.md)
for how the firmware is built.

## Features (planned — see IMPLEMENTATION_CHECKLIST.md)

- Reads HWT3100 ASCII-mode serial output (heading + raw magnetic field)
- Transmits heading via NMEA 2000 PGN 127250 (Vessel Heading)
- Publishes `navigation.headingMagnetic` SignalK deltas
- Configurable heading calibration offset (mounting misalignment)
- In-place on-module magnetic-field calibration commands, from a fixed
  allowlist (`AT+CALI=0/1/2`) — see SPEC.md §8.2 for why this is scoped
  the way it is
- Live serial terminal in the web UI for wiring/troubleshooting
- SensESP-based: WiFi AP/client, web UI configuration, OTA updates

**Not supported, deliberately** (see SPEC.md §9.3): pitch/roll/full
attitude (the HWT3100-TTL/232 is a compass, not an IMU — this is a
hardware limitation, not a deferred feature), and Modbus mode /
`AT+MODE` (a documented hazard — see SPEC.md §1.2).

## Building

Requires [PlatformIO](https://platformio.org/).

```bash
pio run
```

## License

MIT
