#pragma once

#include <stdint.h>

// Where IMU readings go. Feather M4 CAN: one CAN frame per reading
// (imu_output_can_samd.cpp). ESP32-S2 test build: one text line per reading
// on Serial (imu_output_serial_esp32.cpp).

// Call once from setup(), before imuSystemInit().
void imuOutputInit();

// Call once per loop() iteration (housekeeping only -- may be a no-op).
void imuOutputPoll();

// One accelerometer reading, X/Y/Z in mg.
void imuOutputAccel(uint8_t imu, int16_t xMg, int16_t yMg, int16_t zMg);

// One gyroscope reading, X/Y/Z in 0.1 dps.
void imuOutputGyro(uint8_t imu, int16_t xDdps, int16_t yDdps, int16_t zDdps);

// One SFLP game-rotation-vector reading: qx/qy/qz as raw IEEE-754
// half-precision bit patterns, exactly as the sensor's FIFO delivers them
// (qw isn't delivered -- qw = sqrt(max(0, 1 - qx^2 - qy^2 - qz^2))).
void imuOutputQuat(uint8_t imu, uint16_t qx, uint16_t qy, uint16_t qz);
