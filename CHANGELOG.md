# Changelog

All notable changes to this project are documented in this file.

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
