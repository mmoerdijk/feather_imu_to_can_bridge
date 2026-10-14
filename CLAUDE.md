# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

PlatformIO/Arduino firmware (C/C++) that reads up to 10 ST LSM6DSV IMUs over five I2C buses and
streams accel/gyro/orientation data out over CAN. Builds for two boards: the Adafruit Feather M4
CAN Express (env `adafruit_feather_m4_can`), and a cut-down bench-test build for a WEMOS LOLIN S2
Mini (ESP32-S2, env `lolin_s2_mini`, the default env) with only 2 buses / 4 IMUs and Serial text
output instead of CAN. Platform code lives in whole-file-`#if`-guarded `*_esp32.cpp` /
`*_samd.cpp` pairs behind the `i2c_buses.h`, `imu_output.h` and `status_led.h` interfaces;
`imu.cpp` and `main.cpp` are shared.
**`README.md` is the primary reference** — it documents the I2C bus topology, CAN frame
encoding, status LED semantics, configuration `#define`s, and known hardware quirks in detail;
don't duplicate that here, read it before touching `src/imu.cpp` or the CAN code. The Feather
is driven by a sibling hardware repo, `../adapter_board` (KiCad), which breaks out the IMU
connections this firmware talks to.

## Commands

Build / flash / monitor (from this directory; needs the PlatformIO CLI — see README for setup):
```
pio run                 # build (default env: lolin_s2_mini)
pio run -t upload       # build + flash over USB
pio run -e adafruit_feather_m4_can   # build the Feather env
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
- `src/i2c_buses.h` + `_samd.cpp`/`_esp32.cpp` — the I2C buses (`NUM_BUSES`: 5 on the Feather,
  2 on the ESP32) and their pin wiring (`buses[]`), one `TwoWire` per bus. `NUM_IMUS` is
  `2 * NUM_BUSES`.
- `src/imu.h/.cpp` — platform-neutral: the boot-time detection probe, the offline/reconnect
  state machine, and the accel/gyro/quaternion reads. IMU-index → bus/address mapping is
  `sensorBus()`/`sensorAddr()` (IMU 2n at 0x6A, 2n+1 at 0x6B on bus n). Key non-obvious invariant: an IMU slot that
  doesn't respond during the one-time boot probe is permanently excluded from retries for the
  rest of the run (`eligibleForRetry`) — a slot wired up later needs a reset/reflash, not just
  time, to be picked up.
- `src/imu_output.h` — where readings go. ESP32: `imu_output_serial_esp32.cpp` prints one text
  line per reading. Feather: `imu_output_can_samd.cpp`, CAN0 via Adafruit CAN's `CANSAME5x`, with
  the per-IMU sequence counters and the fixed 8-byte frame
  encoding (3×int16 + sequence counter + reserved byte) for accel/gyro/quaternion, one frame per
  reading with no batching. Has a loopback self-verify mode (`CAN_LOOPBACK_VERIFY_ENABLED`) for
  testing without external CAN hardware.
- `src/status_led.h` + `_samd.cpp` (NeoPixel colors) / `_esp32.cpp` (GPIO15 LED, blink patterns
  instead of colors) — health summary (boot-probe → found
  count → running-state color). State transitions are one-way (once orange/red, never back to
  green) — that's intentional, see README.
- `src/lsm6dsv_reg.c` / `include/lsm6dsv_reg.h` — **vendored ST driver, do not hand-edit** (also
  excluded from clang-format). Replace wholesale from ST's source if it needs updating, don't
  patch it in place.

## Gotchas that span files (see README for full detail on each)

- No floating-point `printf`/`snprintf` (`%f` silently prints empty) on the Feather toolchain — use
  `dtostrf()`, as `printStatusTable()` in `imu.cpp` does.
- `Adafruit TinyUSB Library` is force-excluded via `lib_ignore` in `platformio.ini` because
  PlatformIO's dependency scanner pulls it in from a guarded `#include` in `Adafruit_NeoPixel.h`
  even though the `#ifdef` around it is never taken; adding a new library that transitively
  includes NeoPixel can resurface this.
- Feather: don't toggle `pinMode()` on a bus's SDA/SCL after `i2cBusClaimIfIdle()` has claimed
  it (`i2c_buses_samd.cpp`) — a `pinMode`/`pinPeripheral` round-trip after `wire->begin()` breaks I2C on that
  bus even on healthy hardware.
- Feather: bus 4 repurposes `D0`/`D1`, so `Serial1` doesn't work while this firmware runs.
- ESP32 env uses the pioarduino platform; on Windows its first build (toolchain install) must run
  from PowerShell/cmd, not Git Bash — ESP-IDF's installer rejects MSYS.
