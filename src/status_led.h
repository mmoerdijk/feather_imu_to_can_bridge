#pragma once

#include <stdint.h>

// Solid blue: call once at the very start of setup(), before IMU detection.
void ledInit();

// Flashes white `count` times (one per IMU found at boot), then settles to
// solid green. If count is 0, leaves the LED alone instead -- ledUpdateRunState()
// takes over from the very first loop() iteration and blinks red. Blocking --
// intended as a one-time boot-sequence indicator, called once right after the
// boot probe finishes.
void ledFlashDetectedCount(uint8_t count);

// Updates the run-time status color. Worst case wins: noneDetected (blinks
// red -- no IMU ever responded at boot, so anyOffline/anyEverDisconnected
// can never be true on their own) beats anyOffline (solid red) beats
// anyEverDisconnected (solid orange) beats solid green. Only touches the
// LED when the color/blink state actually changes. Call once per loop()
// iteration.
void ledUpdateRunState(bool noneDetected, bool anyOffline, bool anyEverDisconnected);
