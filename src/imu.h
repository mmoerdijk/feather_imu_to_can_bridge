#pragma once

#include <stdint.h>

#include "i2c_buses.h"

// 2 IMUs per I2C bus (one at 0x6A, one at 0x6B): 10 on the Feather, 4 on the
// ESP32-S2 test build.
#define NUM_IMUS (2 * NUM_BUSES)

// Configures the 5 I2C buses and probes all 10 IMU slots once. Call once
// from setup(), after imuOutputInit() and before ledFlashDetectedCount().
void imuSystemInit();

// Number of IMUs that responded during imuSystemInit()'s boot probe -- the
// only ones ever retried or read again afterwards.
uint8_t imuPresentCount();

// Retries offline-but-eligible IMUs on a timer, reads accel/gyro/quaternion
// data as it becomes ready, and hands it to imu_output.h.
// Call once per loop() iteration.
void imuSystemPoll(uint32_t now);

// True if any IMU that connected at boot is currently not responding.
bool imuAnyOffline();

// True if any IMU that connected at boot has disconnected at least once
// since boot (even if it's currently back online).
bool imuAnyEverDisconnected();

// Prints a fixed-width one-line status summary for all eligible IMUs, but
// only actually writes to Serial once per second -- safe to call every
// loop() iteration.
void imuMaybePrintStatusTable(uint32_t now);
