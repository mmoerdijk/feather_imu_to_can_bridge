# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

PlatformIO/Arduino firmware (C/C++) for an Adafruit Feather M4 CAN Express that reads up to 10 ST
LSM6DSV IMUs over five parallel I2C buses and streams accel/gyro/orientation data out over CAN.
**`README.md` is the primary reference** — it documents the I2C bus topology, CAN frame
encoding, status LED semantics, configuration `#define`s, and known hardware quirks in detail;
don't duplicate that here, read it before touching `src/imu.cpp` or `src/can_bus.cpp`. This board
is driven by a sibling hardware repo, `../adapter_board` (KiCad), which breaks out the IMU
connections this firmware talks to.

## Commands

Build / flash / monitor (from this directory; needs the PlatformIO CLI — see README for setup):
```
pio run                 # build
pio run -t upload       # build + flash over USB
pio device monitor      # serial monitor, 115200 baud
pio device list         # list connected boards/ports, useful when upload can't find the device
```

Lint/format — enforced in CI (`.github/workflows/ci.yml`) via the same lefthook config used
locally as the pre-commit hook:
```
npx lefthook run pre-commit --all-files --no-stage-fixed --fail-on-changes
```
which runs `clang-format -i` on `*.{c,h,cpp,hpp}` and `ruff format` + `ruff check --fix` on
`*.py` (there's no Python in `src/`, but the repo's tooling scripts fall under this). Excludes
`src/lsm6dsv_reg.c`/`.h` from clang-format — see below. `ruff.toml` selects `E`, `F`, `I` at a
100-char line length; `.clang-format` is LLVM-based, 2-space indent, Allman braces.

There are no unit tests — `test/` and `lib/` are untouched PlatformIO boilerplate.

## Architecture

- `src/main.cpp` — thin `setup()`/`loop()` orchestration only; the real logic lives in the
  modules below. Read it first to see call order, but expect to edit the modules, not this file.
- `src/imu.h/.cpp` — owns all 5 `TwoWire` instances (default `Wire` + 4 extra SERCOMs), the
  boot-time detection probe, the offline/reconnect state machine, and the accel/gyro/quaternion
  reads. Bus/pin wiring is in `buses[]`; IMU-index → bus/address mapping is in
  `SENSOR_BUS[]`/`SENSOR_ADDR[]` right below it. Key non-obvious invariant: an IMU slot that
  doesn't respond during the one-time boot probe is permanently excluded from retries for the
  rest of the run (`eligibleForRetry`) — a slot wired up later needs a reset/reflash, not just
  time, to be picked up.
- `src/can_bus.h/.cpp` — CAN0 setup via Adafruit CAN's `CANSAME5x`, and the fixed 8-byte frame
  encoding (3×int16 + sequence counter + reserved byte) for accel/gyro/quaternion, one frame per
  reading with no batching. Has a loopback self-verify mode (`CAN_LOOPBACK_VERIFY_ENABLED`) for
  testing without external CAN hardware.
- `src/status_led.h/.cpp` — drives the onboard NeoPixel as a health summary (boot-probe → found
  count → running-state color). State transitions are one-way (once orange/red, never back to
  green) — that's intentional, see README.
- `src/lsm6dsv_reg.c` / `include/lsm6dsv_reg.h` — **vendored ST driver, do not hand-edit** (also
  excluded from clang-format). Replace wholesale from ST's source if it needs updating, don't
  patch it in place.

## Gotchas that span files (see README for full detail on each)

- No floating-point `printf`/`snprintf` (`%f` silently prints empty) on this toolchain — use
  `dtostrf()`, as `printStatusTable()` in `imu.cpp` does.
- `Adafruit TinyUSB Library` is force-excluded via `lib_ignore` in `platformio.ini` because
  PlatformIO's dependency scanner pulls it in from a guarded `#include` in `Adafruit_NeoPixel.h`
  even though the `#ifdef` around it is never taken; adding a new library that transitively
  includes NeoPixel can resurface this.
- Don't toggle `pinMode()` on a bus's SDA/SCL after `claimBusIfIdle()` has claimed it
  (`imu.cpp`) — a `pinMode`/`pinPeripheral` round-trip after `wire->begin()` breaks I2C on that
  bus even on healthy hardware.
- Bus 4 repurposes `D0`/`D1`, so `Serial1` doesn't work while this firmware runs.
