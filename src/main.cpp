#include <Arduino.h>
#include <Wire.h>
#include <wiring_private.h> // pinPeripheral()

extern "C" {
#include "lsm6dsv_reg.h"
}

// SDO/SA0 tied low -> 0x6A, tied high -> 0x6B (7-bit I2C address)
#define LSM6DSV_ADDR_L (LSM6DSV_I2C_ADD_L >> 1)
#define LSM6DSV_ADDR_H (LSM6DSV_I2C_ADD_H >> 1)

#define NUM_BUSES 5
#define NUM_IMUS 10

// Bus 0 uses the board's default STEMMA QT Wire (SERCOM2, D21/D22).
// Buses 1-4 repurpose SERCOM3/4/0/5, whose pads land on otherwise-free
// header pins on this board (verified against the Feather M4 CAN schematic).
// None of these pins carry onboard pull-ups -- every bus needs external
// pull-up resistors on SDA/SCL.
TwoWire busWire1(&sercom3, 12, 13);        // D12 (SDA), D13 (SCL)
TwoWire busWire2(&sercom4, 16, 17);        // A2  (SDA), A3  (SCL)
TwoWire busWire3(&sercom0, 18, 15);        // A4  (SDA), A1  (SCL)
TwoWire busWire4(&sercom5, 1, 0);          // D1  (SDA), D0  (SCL) -- takes over Serial1's pins

struct bus_desc_t
{
  TwoWire *wire;
  uint8_t pinSDA;
  uint8_t pinSCL;
  EPioType muxType; // mux function TwoWire::begin() needs forced onto pinSDA/pinSCL
};

static bus_desc_t buses[NUM_BUSES] = {
    {&Wire,     21, 22, PIO_SERCOM},     // default mux already correct
    {&busWire1, 12, 13, PIO_SERCOM},     // SERCOM3 is the primary mux on D12/D13
    {&busWire2, 16, 17, PIO_SERCOM_ALT}, // SERCOM4 is the alt mux on A2/A3
    {&busWire3, 18, 15, PIO_SERCOM_ALT}, // SERCOM0 is the alt mux on A4/A1
    {&busWire4, 1,  0,  PIO_SERCOM},     // SERCOM5 is the primary mux on D0/D1
};

struct i2c_target_t
{
  TwoWire *wire;
  uint8_t addr;
};

#define OFFLINE_RETRY_INTERVAL_MS 2000

struct imu_slot_t
{
  i2c_target_t target;
  stmdev_ctx_t ctx;
  bool present;
  uint32_t nextRetryMs; // millis() timestamp of the next detect/re-detect attempt
};

static i2c_target_t imuTargets[NUM_IMUS];
static imu_slot_t imus[NUM_IMUS];

// bus index and I2C address for each of the 10 IMUs (2 per bus)
static const uint8_t SENSOR_BUS[NUM_IMUS]  = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};
static const uint8_t SENSOR_ADDR[NUM_IMUS] = {
    LSM6DSV_ADDR_L, LSM6DSV_ADDR_H,
    LSM6DSV_ADDR_L, LSM6DSV_ADDR_H,
    LSM6DSV_ADDR_L, LSM6DSV_ADDR_H,
    LSM6DSV_ADDR_L, LSM6DSV_ADDR_H,
    LSM6DSV_ADDR_L, LSM6DSV_ADDR_H,
};

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  i2c_target_t *target = (i2c_target_t *)handle;

  target->wire->beginTransmission(target->addr);
  target->wire->write(reg);
  target->wire->write(bufp, len);
  return target->wire->endTransmission();
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  i2c_target_t *target = (i2c_target_t *)handle;

  target->wire->beginTransmission(target->addr);
  target->wire->write(reg);
  if (target->wire->endTransmission(false) != 0)
  {
    return -1;
  }

  uint16_t received = target->wire->requestFrom(target->addr, len);
  if (received != len)
  {
    return -1;
  }

  for (uint16_t i = 0; i < len; i++)
  {
    bufp[i] = target->wire->read();
  }
  return 0;
}

static void platform_delay(uint32_t ms)
{
  delay(ms);
}

// A bus with no pull-ups (nothing wired to it yet, or a dead connection)
// never reaches a valid idle-high state. The vendor SERCOM I2C driver's
// transmission-start wait loop has no timeout and spins forever in that
// case, so this check must happen *before* any blocking Wire call --
// otherwise one unwired bus permanently freezes the whole sketch.
static bool busIsIdle(bus_desc_t &bus)
{
  pinMode(bus.pinSDA, INPUT);
  pinMode(bus.pinSCL, INPUT);
  delayMicroseconds(50);
  bool idle = digitalRead(bus.pinSDA) == HIGH && digitalRead(bus.pinSCL) == HIGH;

  // Restore the SERCOM mux pinMode() just switched away from.
  pinPeripheral(bus.pinSDA, bus.muxType);
  pinPeripheral(bus.pinSCL, bus.muxType);
  return idle;
}

enum imu_init_result_t
{
  IMU_INIT_OK,
  IMU_INIT_BUS_NOT_READY,
  IMU_INIT_NOT_DETECTED,
};

// Probes and (re)configures one IMU. Safe to call repeatedly to detect a
// sensor that was missing at boot or that dropped off the bus and came back.
static imu_init_result_t initImu(uint8_t i)
{
  if (!busIsIdle(buses[SENSOR_BUS[i]]))
  {
    return IMU_INIT_BUS_NOT_READY;
  }

  uint8_t whoamI = 0;
  lsm6dsv_device_id_get(&imus[i].ctx, &whoamI);
  if (whoamI != LSM6DSV_ID)
  {
    return IMU_INIT_NOT_DETECTED;
  }

  lsm6dsv_sw_reset(&imus[i].ctx);

  // Block Data Update: registers only updated after MSB and LSB read
  lsm6dsv_block_data_update_set(&imus[i].ctx, PROPERTY_ENABLE);

  lsm6dsv_xl_full_scale_set(&imus[i].ctx, LSM6DSV_4g);
  lsm6dsv_gy_full_scale_set(&imus[i].ctx, LSM6DSV_2000dps);

  lsm6dsv_xl_data_rate_set(&imus[i].ctx, LSM6DSV_ODR_AT_120Hz);
  lsm6dsv_gy_data_rate_set(&imus[i].ctx, LSM6DSV_ODR_AT_120Hz);

  return IMU_INIT_OK;
}

static const char *initResultMessage(imu_init_result_t result)
{
  switch (result)
  {
  case IMU_INIT_OK:
    return ": initialized";
  case IMU_INIT_BUS_NOT_READY:
    return ": OFFLINE (bus not ready -- check wiring/pull-ups)";
  default:
    return ": OFFLINE (not detected)";
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }

  for (uint8_t b = 0; b < NUM_BUSES; b++)
  {
    buses[b].wire->begin();
    // TwoWire::begin() re-applies each pin's *default* mux, which is wrong
    // for pins whose I2C function is not their default (A1-A4, D12, D13) --
    // force the correct SERCOM mux back onto them here.
    pinPeripheral(buses[b].pinSDA, buses[b].muxType);
    pinPeripheral(buses[b].pinSCL, buses[b].muxType);
    buses[b].wire->setClock(100000);
  }

  for (uint8_t i = 0; i < NUM_IMUS; i++)
  {
    imuTargets[i].wire = buses[SENSOR_BUS[i]].wire;
    imuTargets[i].addr = SENSOR_ADDR[i];

    imus[i].target = imuTargets[i];
    imus[i].ctx.write_reg = platform_write;
    imus[i].ctx.read_reg = platform_read;
    imus[i].ctx.mdelay = platform_delay;
    imus[i].ctx.handle = &imuTargets[i];
    imus[i].present = false;
    imus[i].nextRetryMs = 0;
  }

  platform_delay(10); // sensor boot time

  for (uint8_t i = 0; i < NUM_IMUS; i++)
  {
    imu_init_result_t result = initImu(i);
    imus[i].present = (result == IMU_INIT_OK);
    imus[i].nextRetryMs = millis() + OFFLINE_RETRY_INTERVAL_MS;

    Serial.print("IMU");
    Serial.print(i);
    Serial.println(initResultMessage(result));
  }
}

void loop()
{
  uint32_t now = millis();

  for (uint8_t i = 0; i < NUM_IMUS; i++)
  {
    if (!imus[i].present)
    {
      // Periodically re-probe an offline sensor so it comes back automatically
      // if it was plugged in late or recovered from a bus glitch.
      if ((int32_t)(now - imus[i].nextRetryMs) >= 0)
      {
        imus[i].nextRetryMs = now + OFFLINE_RETRY_INTERVAL_MS;
        imu_init_result_t result = initImu(i);
        imus[i].present = (result == IMU_INIT_OK);

        Serial.print("IMU");
        Serial.print(i);
        Serial.println(imus[i].present ? ": back online" : initResultMessage(result));
      }
      continue;
    }

    lsm6dsv_data_ready_t drdy;
    if (lsm6dsv_flag_data_ready_get(&imus[i].ctx, &drdy) != 0)
    {
      // Lost communication with a previously-working sensor -- mark it
      // offline so it gets reported and retried like any other missing IMU.
      imus[i].present = false;
      imus[i].nextRetryMs = now + OFFLINE_RETRY_INTERVAL_MS;

      Serial.print("IMU");
      Serial.print(i);
      Serial.println(": OFFLINE (lost communication)");
      continue;
    }

    if (!drdy.drdy_xl && !drdy.drdy_gy)
    {
      continue;
    }

    int16_t data_raw[3];

    if (drdy.drdy_xl)
    {
      lsm6dsv_acceleration_raw_get(&imus[i].ctx, data_raw);

      Serial.print("IMU");
      Serial.print(i);
      Serial.print(" Accel [mg] X: ");
      Serial.print(lsm6dsv_from_fs4_to_mg(data_raw[0]));
      Serial.print(" Y: ");
      Serial.print(lsm6dsv_from_fs4_to_mg(data_raw[1]));
      Serial.print(" Z: ");
      Serial.println(lsm6dsv_from_fs4_to_mg(data_raw[2]));
    }

    if (drdy.drdy_gy)
    {
      lsm6dsv_angular_rate_raw_get(&imus[i].ctx, data_raw);

      Serial.print("IMU");
      Serial.print(i);
      Serial.print(" Gyro [mdps] X: ");
      Serial.print(lsm6dsv_from_fs2000_to_mdps(data_raw[0]));
      Serial.print(" Y: ");
      Serial.print(lsm6dsv_from_fs2000_to_mdps(data_raw[1]));
      Serial.print(" Z: ");
      Serial.println(lsm6dsv_from_fs2000_to_mdps(data_raw[2]));
    }
  }
}
