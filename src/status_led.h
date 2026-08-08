#pragma once

#include <stdint.h>

// Solid blue: call once at the very start of setup(), before IMU detection.
void ledInit();

// Flashes white `count` times (one per IMU found at boot), then settles to
// solid green. Blocking -- intended as a one-time boot-sequence indicator,
// called once right after the boot probe finishes.
void ledFlashDetectedCount(uint8_t count);

// Updates the run-time status color: red if anyOffline, else orange if
// anyEverDisconnected, else green. Only touches the LED when the color
// actually changes. Call once per loop() iteration.
void ledUpdateRunState(bool anyOffline, bool anyEverDisconnected);
