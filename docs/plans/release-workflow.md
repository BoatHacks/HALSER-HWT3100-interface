# Implementation Plan: CI + cut-release GitHub Actions workflows

## Overview

Adds a build/test CI workflow and a `workflow_dispatch` release-cutting
workflow adapted from the `cut-release.yml` pattern used in the user's
SignalK npm plugins (`BoatHacks/signalk-stowage-mgmt`), reworked for this
repo's PlatformIO/C++ structure (no `package.json`, no npm). Also builds a
single merged flashable firmware image and attaches it to the release.

## Approach

- **Version source of truth**: a plain `VERSION` file (analogous to
  `package.json`'s `version` field), checked against the workflow's
  `version` input.
- **CHANGELOG.md**: added, using the same `## [x.y.z] - date` heading
  format the reference workflow's `grep`/`awk`-free shell extraction
  expects, seeded with a `0.1.0` entry summarizing everything shipped so
  far.
- **CI gate**: added `.github/workflows/ci.yml` (build `env:halser` +
  `pio test -e native`) since none existed; `cut-release.yml` checks this
  workflow's conclusion for the release commit before publishing, mirroring
  the reference workflow's "Plugin CI" gate.
- **Build/test steps**: `npm ci`/`npm run build`/`npm test`/`npm publish`
  replaced with PlatformIO equivalents (`pio run -e halser`,
  `pio test -e native`); no publish step (no package registry for
  firmware).
- **Merged firmware image**: new step uses the `esptool.py` bundled with
  PlatformIO's `tool-esptoolpy` package (already downloaded as part of the
  `espressif32` platform on `pio run`) to `merge_bin` bootloader +
  partition table + app into one flashable image at the standard ESP32-C3
  offsets (`0x0000`/`0x8000`/`0x10000`), flash params matching
  `esp32-c3-devkitm-1`'s board definition (`qio`, 80MHz, 4MB). Verified
  locally against existing `.pio/build/halser/*.bin` artifacts.
- **Release asset**: merged binary is uploaded via `gh release create`'s
  asset-file argument, named
  `halser-hwt3100-interface-<version>-merged.bin`.

## Files to Create/Modify

- `.github/workflows/ci.yml`
- `.github/workflows/cut-release.yml`
- `VERSION`
- `CHANGELOG.md`
