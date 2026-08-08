#pragma once

#include <stdint.h>

#define NUM_IMUS 10

// Configures the 5 I2C buses and probes all 10 IMU slots once. Call once
// from setup(), after canBusInit() and before ledFlashDetectedCount().
void imuSystemInit();

// Number of IMUs that responded during imuSystemInit()'s boot probe -- the
// only ones ever retried or read again afterwards.
uint8_t imuPresentCount();

// Retries offline-but-eligible IMUs on a timer, reads accel/gyro/quaternion
// data as it becomes ready, and sends it over CAN (via sendCanFrame()).
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
