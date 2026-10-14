#if defined(ARDUINO_ARCH_ESP32)

#include "i2c_buses.h"
#include <Arduino.h>

// WEMOS LOLIN S2 Mini (ESP32-S2) test build: the S2 has 2 hardware I2C
// controllers, so it gets 2 buses (4 IMUs) instead of the Feather's 5.
//
// GPIO9/8 and GPIO3/4 are plain GPIOs on this board (no strapping, flash/PSRAM, USB or
// LED function). As on the Feather, none of them carry pull-ups on the board
// -- those come from the IMU boards.
struct bus_desc_t
{
  TwoWire *wire;
  uint8_t pinSDA;
  uint8_t pinSCL;
  bool claimed; // true once i2cBusClaimIfIdle() has called wire->begin() on this bus
};

static bus_desc_t buses[NUM_BUSES] = {
    {&Wire, 9, 8, false},  // bus 0: GPIO9 (SDA), GPIO8 (SCL)
    {&Wire1, 3, 4, false}, // bus 1: GPIO3 (SDA), GPIO4 (SCL)
};

static void beginBus(bus_desc_t &bus) { bus.wire->begin(bus.pinSDA, bus.pinSCL, I2C_CLOCK_HZ); }

// Unlike the Feather's SERCOM driver, the ESP32 I2C driver has hardware
// timeouts, so a bus without pull-ups can't hang the sketch forever -- but
// it would still stall every transaction for the full timeout. Checking for
// idle-high first (INPUT_PULLDOWN, so a floating pin reads a deterministic
// LOW -- see i2c_buses_samd.cpp) keeps an unwired bus out of the picture
// entirely and gives a clearer "bus not ready" message.
bool i2cBusClaimIfIdle(uint8_t busIndex)
{
  bus_desc_t &bus = buses[busIndex];
  if (bus.claimed)
  {
    return true;
  }

  pinMode(bus.pinSDA, INPUT_PULLDOWN);
  pinMode(bus.pinSCL, INPUT_PULLDOWN);
  delayMicroseconds(50);
  bool idle = digitalRead(bus.pinSDA) == HIGH && digitalRead(bus.pinSCL) == HIGH;
  if (!idle)
  {
    return false;
  }

  beginBus(bus); // reconfigures both pins for I2C, dropping the pull-downs
  bus.claimed = true;
  return true;
}

TwoWire *i2cBusWire(uint8_t busIndex) { return buses[busIndex].wire; }

// A slave that was interrupted mid-transaction can be left holding SDA low
// indefinitely, and a plain STOP can't fix that. Only a master-generated
// clock train gives the slave a chance to finish whatever bit it's stuck on
// and let go; that's what this does, followed by a manufactured STOP. Both
// lines are driven open-drain, so the IMU board's pull-ups do the rising.
void i2cBusRecover(uint8_t busIndex)
{
  bus_desc_t &bus = buses[busIndex];
  bus.wire->end();

  digitalWrite(bus.pinSCL, HIGH);
  digitalWrite(bus.pinSDA, HIGH);
  pinMode(bus.pinSCL, OUTPUT_OPEN_DRAIN);
  pinMode(bus.pinSDA, OUTPUT_OPEN_DRAIN); // released (high), so SDA still reads the slave

  for (uint8_t i = 0; i < 9 && digitalRead(bus.pinSDA) == LOW; i++)
  {
    digitalWrite(bus.pinSCL, LOW);
    delayMicroseconds(5);
    digitalWrite(bus.pinSCL, HIGH);
    delayMicroseconds(5);
  }

  // Manufacture a STOP condition: SDA rising while SCL is high.
  digitalWrite(bus.pinSCL, LOW);
  delayMicroseconds(5);
  digitalWrite(bus.pinSDA, LOW);
  delayMicroseconds(5);
  digitalWrite(bus.pinSCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(bus.pinSDA, HIGH);
  delayMicroseconds(5);

  beginBus(bus);
}

#endif // ARDUINO_ARCH_ESP32
