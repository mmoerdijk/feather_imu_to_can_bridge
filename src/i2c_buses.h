#pragma once

#include <Wire.h>
#include <stdint.h>

// Platform-specific I2C bus layer, 2 IMUs per bus. Feather M4 CAN: 5 buses,
// one SERCOM each (i2c_buses_samd.cpp). ESP32-S2 test build: 2 buses, one per
// hardware I2C controller (i2c_buses_esp32.cpp).

#if defined(ARDUINO_ARCH_ESP32)
#define NUM_BUSES 2
#else
#define NUM_BUSES 5
#endif

// LSM6DSV supports up to 1000000 (Fast-mode+); 400000 (Fast-mode) is a more
// forgiving next step up from 100000 given the pull-up/wiring situation.
// Each IMU board carries 10k pull-ups, so a bus with a single IMU (or long
// jumper leads) has weak edges -- drop to 100000 if a lone IMU won't answer.
#define I2C_CLOCK_HZ 400000

// Claims `bus` for I2C the first time its SDA/SCL are both seen idle-high
// (i.e. something with pull-ups is attached). Returns true if the bus is
// (now or already) claimed, false if it isn't safe to use yet.
bool i2cBusClaimIfIdle(uint8_t bus);

// Returns the TwoWire for `bus`. Only valid once i2cBusClaimIfIdle(bus) has
// returned true.
TwoWire *i2cBusWire(uint8_t bus);

// Clocks a wedged slave off SDA and issues a STOP (NXP UM10204 S3.1.16).
// Call after a failed transaction on a previously-working bus.
void i2cBusRecover(uint8_t bus);
