# Changelog

All notable changes to this project are documented in this file.

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
