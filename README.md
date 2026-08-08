# IMU Reader — Feather M4 CAN

Firmware for an Adafruit Feather M4 CAN Express that reads up to 10 ST
LSM6DSV IMUs over five parallel I2C buses, reports per-sensor connection
health over USB Serial, and shows an at-a-glance health summary on the
board's onboard NeoPixel.

This board sits on a custom adapter/carrier PCB (see the sibling
`adapter_board` repo) that breaks out 10 AYF530435 FPC connectors, two per
I2C bus, for the IMU sensor boards.

## Contents

- [Hardware overview](#hardware-overview)
- [I2C bus topology](#i2c-bus-topology)
- [Status LED](#status-led)
- [Serial output](#serial-output)
- [Setting up PlatformIO (VS Code)](#setting-up-platformio-vs-code)
- [Building, flashing, monitoring](#building-flashing-monitoring)
- [Configuration](#configuration)
- [How detection & reconnect work](#how-detection--reconnect-work)
- [Known quirks / gotchas](#known-quirks--gotchas)
- [Repo layout](#repo-layout)
- [Not implemented yet](#not-implemented-yet)

## Hardware overview

- **MCU board**: Adafruit Feather M4 CAN Express (SAME51J19A, Cortex-M4,
  120 MHz). Datasheet/pinout: `docs/Adafruit Feather M4 Express CAN
  Pinout.pdf`.
- **Sensors**: up to 10x ST **LSM6DSV** 6-axis IMU (accel + gyro).
  Datasheet: `docs/lsm6dsv.pdf`. Driver: ST's own register-level driver,
  vendored in `src/lsm6dsv_reg.c` / `include/lsm6dsv_reg.h` (do not hand-edit
  — it's the ST source, regenerate/replace wholesale if it ever needs
  updating).
- **Status indicator**: the Feather's onboard NeoPixel (single WS2812-style
  RGB LED, `PIN_NEOPIXEL` on the board).
- Each IMU's I2C address is set in hardware by strapping its SA0/SDO pin:
  tied low → `0x6A`, tied high → `0x6B`. Two IMUs (one at each address)
  share each physical I2C bus.

## I2C bus topology

The M4's default `Wire` only gives you one I2C bus. To talk to 10 IMUs at
once, this firmware repurposes four additional SERCOM peripherals as extra
`TwoWire` instances, so there are **5 physical I2C buses, 2 IMUs each**:

| Bus | `TwoWire` object | SDA pin | SCL pin | SERCOM | IMU slots |
|-----|------------------|---------|---------|--------|-----------|
| 0 | `Wire` (onboard STEMMA QT) | D21 | D22 | SERCOM2 | IMU0 (0x6A), IMU1 (0x6B) |
| 1 | `busWire1` | D12 | D13 | SERCOM3 | IMU2 (0x6A), IMU3 (0x6B) |
| 2 | `busWire2` | A2 | A3 | SERCOM4 | IMU4 (0x6A), IMU5 (0x6B) |
| 3 | `busWire3` | A4 | A1 | SERCOM0 | IMU6 (0x6A), IMU7 (0x6B) |
| 4 | `busWire4` | D1 | D0 | SERCOM5 | IMU8 (0x6A), IMU9 (0x6B) |

Notes:
- Bus 4 repurposes `D0`/`D1`, which are normally `Serial1`'s RX/TX pins —
  don't expect `Serial1` to work while this firmware runs.
- **None of buses 1-4 have onboard pull-ups.** Every bus (including bus 0,
  which also has no pull-ups on this board revision) needs external
  pull-up resistors on both SDA and SCL for the bus to reach a valid
  idle-high state. A bus with no pull-ups will report every IMU on it as
  `OFFLINE (bus not ready -- check wiring/pull-ups)` — that's this specific
  failure mode, not a generic "not detected".
- Bus/pin assignments live in `buses[]` in `src/main.cpp`; which IMU index
  maps to which bus/address lives in `SENSOR_BUS[]` / `SENSOR_ADDR[]`
  right below it.

## Status LED

The onboard NeoPixel gives a health summary with no terminal required:

| Phase | Color | Meaning |
|-------|-------|---------|
| Boot / detecting | **Blue** (solid) | Probing all 10 IMU slots at startup. |
| Just after boot | **White** (flashing) | Flashes once per IMU that was found at boot — count the flashes to know how many connected. |
| Running | **Green** | All IMUs that were found at boot are currently online and have never dropped. |
| Running | **Orange** | Everything is currently online, but at least one IMU has disconnected and reconnected at some point since boot. |
| Running | **Red** | At least one IMU that was found at boot is currently offline right now. Takes priority over orange. |

Once an IMU has disconnected at least once, the LED will never go back to
green for the rest of the run (it settles to orange instead) — this is
intentional, it's a "has this run been perfectly clean" indicator, not just
"is everything currently fine".

## Serial output

Open a serial monitor at **115200 baud**. At boot you get one line per IMU
slot, in order:

```
IMU0: initialized
IMU1: initialized
IMU2: OFFLINE (bus not ready -- check wiring/pull-ups)
...
```

After that, once per second (`STATUS_PRINT_INTERVAL_MS`), a single
fixed-width status line is printed for every IMU that *did* respond at
boot (others are permanently omitted — see below):

```
IMU0  ONLINE  drops=  0 z= -444.6mg | IMU1  ONLINE  drops=  0 z=  204.5mg
```

`drops` counts how many times that IMU has gone from online to offline
since boot. `z` is the most recent accelerometer Z-axis reading in mg (not
printed live per-sample — only this cached table).

Between status lines, transition events are still printed immediately as
they happen, e.g. `IMU0: OFFLINE (lost communication)` or `IMU0: back
online`.

## Setting up PlatformIO (VS Code)

This project uses [PlatformIO](https://platformio.org/), not the Arduino
IDE. If you don't have it yet:

1. Install [Visual Studio Code](https://code.visualstudio.com/).
2. Open VS Code, go to the Extensions view (`Ctrl+Shift+X`), search for
   **"PlatformIO IDE"** (publisher: PlatformIO), and click **Install**.
   - This pulls in its own Python environment and toolchains automatically
     — you don't need to install Python, GCC, or any Arduino cores by
     hand. First-time setup can take a few minutes.
   - VS Code will prompt you to reload after installation; do that.
3. A new alien-head **PlatformIO icon** appears in the VS Code activity bar
   (left-hand sidebar) once installed.
4. In VS Code: **File → Open Folder…** and select this `imu_reader`
   folder (the one containing `platformio.ini`). PlatformIO auto-detects
   the project from `platformio.ini` — no manual project setup needed.
5. The first time you open the project, PlatformIO downloads the
   `atmelsam` platform, the Arduino framework core for SAMD/Adafruit
   boards, and the libraries listed under `lib_deps` in `platformio.ini`
   (currently just `Adafruit NeoPixel`). Watch the PlatformIO output panel
   at the bottom for progress.

You don't need the separate Arduino IDE or `arduino-cli` installed — the
PlatformIO extension is self-contained.

## Building, flashing, monitoring

All of this is available both as PlatformIO's own VS Code toolbar buttons
(bottom status bar: checkmark = build, arrow = upload, plug icon =
monitor) and as CLI commands (`pio`, available in any terminal once the
PlatformIO IDE extension has installed its CLI, or via VS Code's
integrated terminal):

```sh
pio run                 # build
pio run -t upload       # build + flash over USB
pio device monitor      # open a serial monitor (115200 baud, matches platformio.ini)
```

Plug the Feather M4 CAN in via USB before uploading. Uploading uses the
board's native USB bootloader (`sam-ba` protocol, 1200bps-touch reset) — no
physical reset button press should be needed.

**If upload fails with "No device found on COM_"**: this SAM-BA upload
sequence is known to be flaky — it fails once or twice before succeeding
on a bare retry more often than not. Just run `pio run -t upload` again.
If it persists, check nothing else (another serial monitor, another
PlatformIO instance) is holding the COM port open, and check the board is
actually enumerated (`pio device list`).

## Configuration

A handful of `#define`s near the top of `src/main.cpp` control behavior —
change and reflash, there's no runtime config:

| Define | Default | Effect |
|--------|---------|--------|
| `I2C_CLOCK_HZ` | `400000` | I2C bus clock speed (Hz) for every bus. LSM6DSV supports up to `1000000` (Fast-mode+); `400000` is Fast-mode. Standard-mode is `100000`. |
| `OFFLINE_RETRY_INTERVAL_MS` | `2000` | How often an eligible-but-currently-offline IMU is re-probed. |
| `STATUS_PRINT_INTERVAL_MS` | `1000` | How often the combined status line is printed. |
| `AUTO_RECONNECT_ENABLED` | `true` | Set to `false` to disable all reconnect attempts — an IMU that drops just stays `OFFLINE` until you flip this back and reflash. |

## How detection & reconnect work

- At boot, **every** IMU slot (0-9) is probed exactly once, in order,
  regardless of whether its bus has ever been used before.
- **Only IMUs that responded at boot are ever retried again.** A slot
  that fails at boot (`eligibleForRetry = false`) is permanently excluded
  from the retry loop, the status table, and further I2C traffic for the
  rest of the run — even if you plug something into it later, it will not
  be picked up without a reset/reflash. This is deliberate: it keeps the
  ongoing status view focused on the sensors that are actually wired up,
  rather than retrying 10 slots forever when you might only have 2
  connected.
- IMUs that *did* respond at boot keep auto-reconnecting on the
  `OFFLINE_RETRY_INTERVAL_MS` timer if they drop later (unless
  `AUTO_RECONNECT_ENABLED` is `false`).
- **If you wire up a new sensor and want it picked up, reset or reflash
  the board** so the boot probe runs again.

## Known quirks / gotchas

- **No `%f` in `printf`/`snprintf` on this toolchain.** This Arduino core's
  libc has no floating-point printf support by default — `%f`/`%7.1f`
  silently produces an empty string, not a compile or runtime error. Use
  `dtostrf()` (declared via `#include <avr/dtostrf.h>`) to format floats
  instead, as `printStatusTable()` does.
- **Don't toggle `pinMode()` on a bus's SDA/SCL pins after it's been
  claimed.** Once a bus's `wire->begin()` has been called
  (`claimBusIfIdle()` in `src/main.cpp`), doing a `pinMode(INPUT)` →
  `pinPeripheral(restore)` round-trip on those pins breaks I2C
  communication on that bus, even on otherwise-healthy hardware. This was
  the root cause of an earlier "no connection" bug and is why the idle
  check only ever runs once, before a bus's first claim.
- **`Adafruit TinyUSB Library` is deliberately excluded** via `lib_ignore`
  in `platformio.ini`. `Adafruit_NeoPixel.h` has a guarded `#include
  <Adafruit_TinyUSB.h>` for boards that use the TinyUSB stack; PlatformIO's
  Library Dependency Finder scans `#include` lines as plain text and pulls
  that library in regardless of the surrounding (never-taken) `#ifdef`,
  and it fails to build without a USB stack explicitly selected. If you
  add a library later and see it suddenly fail to build with `Adafruit
  TinyUSB` compile errors, this is why.
- **Bus 4 steals `Serial1`'s pins** (D0/D1) — see the topology table above.

## Repo layout

```
src/main.cpp            all firmware logic (bus setup, IMU polling, status LED, Serial reporting)
src/lsm6dsv_reg.c        vendored ST LSM6DSV register driver (do not hand-edit)
include/lsm6dsv_reg.h    ^ its header
docs/lsm6dsv.pdf                                 LSM6DSV datasheet
docs/Adafruit Feather M4 Express CAN Pinout.pdf   Feather M4 CAN pinout reference
platformio.ini           board/framework/library config
```

## Not implemented yet

Sending IMU data out over the Feather's onboard CAN transceiver is planned
but not yet implemented in this codebase.
