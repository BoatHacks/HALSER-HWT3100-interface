# Changelog

All notable changes to this project are documented in this file.

## [0.3.0] - 2026-08-17

### Added

- HWT3100 UART baud rate is now auto-detected at boot (tries 115200,
  then 9600, then 460800) if not already known, and switchable live via
  config (`AT+UART`) — the firmware reconfigures its own UART to match
  immediately after sending the command.
- Raw magnetic field published as SignalK deltas
  (`sensors.hwt3100.magneticField.x/y/z`), independently toggleable,
  off by default.
- `navigation.headingMagnetic` and `navigation.rateOfTurn` are now
  independently toggleable in the config UI, on top of the existing
  SignalK master enable switch — matching the N2K side's existing
  master-plus-per-PGN pattern.

### Changed

- The "pre-configure the module to 115200 baud before wiring to
  HALSER" step is now a recommendation, not a requirement —
  auto-detection resolves the chicken-and-egg problem that used to
  make it mandatory.

### Fixed

- PGN 127250/127251 are now correctly declared via
  `ExtendTransmitMessages()`, so PGN 126464 ("PGN List") queries
  report them. Previously only the NMEA2000-library's own automatic
  boilerplate PGNs were reported, which some MFDs/tools may have used
  to decide this device isn't a valid heading/rate-of-turn source even
  though the data was genuinely on the bus.

## [0.2.2] - 2026-08-17

### Removed

- The temporary "TEST: Ping SignalK server" `UIButton` diagnostic
  added in 0.2.1. Testing on real hardware confirmed the suspected
  upstream SensESP bug: the button never rendered anywhere in the web
  UI. No longer needed now that the finding is confirmed; also drops
  the `HTTPClient` dependency it pulled in.

## [0.2.1] - 2026-08-17

### Added

- Config UI/SPEC now recommend `AT+PRATE=100` (10 datagrams/second) as
  the practical minimum output rate for usable heading/rate-of-turn
  resolution.
- **Temporary diagnostic build**: adds a `UIButton` ("TEST: Ping
  SignalK server") that fires an HTTP GET to the SignalK server on
  click, to empirically test a suspected upstream SensESP bug (see
  `docs/plans/uibutton-investigation.md`). Not a permanent feature —
  will be removed in a follow-up release once tested.

### Changed

- Calibration action checkboxes reworded for clarity ("Calibration
  1/3: Start" etc.), explicitly stating the check-and-Save,
  auto-resets mechanic on all three instead of just the first.

## [0.2.0] - 2026-08-17

### Added

- Rate of turn now published to SignalK as `navigation.rateOfTurn`
  (rad/s, +ve = starboard), alongside the existing N2K PGN 127251.
- `AT+FILT` (on-module output smoothing filter) exposed as persisted
  config: 0 = off, 1-999 = filter strength.
- `AT+PRATE` (on-module output data rate) exposed as persisted config,
  auto-detected from the sensor at first boot if not yet known, so the
  firmware never has to guess a value that could silence the module's
  continuous data stream.

### Changed

- Firmware now requires the HWT3100 module to be pre-configured to
  115200 baud (`AT+UART=1`) before wiring to HALSER, and only supports
  the HWT3100-**TTL** variant (not -232/-485).

### Fixed

- Corrected the documented/implemented `AT+FILT` range to `[0, 999]`
  (no `AT+FILT=1000` exists in the real command set).

### Security

- Documented `AT+MRATE` and `AT+ID` as permanently excluded, alongside
  `AT+MODE` — unconfirmed bricking risk, and both are meaningless
  without the already-excluded Modbus mode.

## [0.1.1] - 2026-08-16

### Added

- Release workflow now also attaches an OTA-flashable image
  (`*-ota.bin`, the plain app partition with no bootloader/partition
  table) alongside the merged USB-flash image, for use with SensESP's
  built-in `ArduinoOTA` support.

### Fixed

- The release workflow's merged flashable firmware image was built with
  `--flash_mode qio`, but this board actually compiles/boots in DIO mode.
  `esptool merge_bin` unconditionally rewrites the flash-mode field in the
  image headers, so the `v0.1.0` release asset silently claimed QIO and
  hung in a `TG0WDT_SYS_RST` boot loop on real hardware. The workflow now
  reads the actual flash mode out of the compiled `bootloader.bin` header
  instead of hardcoding it. **The `v0.1.0` merged binary is broken — do
  not flash it; use `v0.1.1` or later.**

## [0.1.0] - 2026-08-16

### Added

- WitMotion HWT3100-TTL ASCII protocol parser and dedicated serial I/O task.
- Calibration command handling (start/end/clear) with an explicit write
  allowlist — `AT+MODE` is never sent, under any circumstance.
- MFD-triggered calibration over N2K (PGN 130850 command / PGN 130851 ack).
- B&G Precision-9 N2K device identity emulation.
- PGN 127250 (heading) and PGN 127251 (computed rate of turn, windowed
  least-squares regression) senders.
- SignalK output for magnetic heading and rate of turn, with `meta.timeout`
  staleness signaling instead of an active notification.
- Web config UI serial terminal for observing raw HWT3100 output.
