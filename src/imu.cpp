#include <Arduino.h>
#include <Wire.h>
#include <math.h>  // NAN, isnan(), lroundf()
#include <stdio.h> // snprintf()
#if defined(ARDUINO_ARCH_SAMD)
#include <avr/dtostrf.h> // dtostrf() -- this core's own float-to-string emulation,
                         // since its libc snprintf has no %f support
#endif
#include "i2c_buses.h"
#include "imu.h"
#include "imu_output.h"

extern "C"
{
#include "lsm6dsv_reg.h"
}

// SDO/SA0 tied low -> 0x6A, tied high -> 0x6B (7-bit I2C address)
#define LSM6DSV_ADDR_L (LSM6DSV_I2C_ADD_L >> 1)
#define LSM6DSV_ADDR_H (LSM6DSV_I2C_ADD_H >> 1)

struct i2c_target_t
{
  uint8_t bus; // index into i2c_buses.h's buses -- see i2cBusWire()
  uint8_t addr;
};

#define OFFLINE_RETRY_INTERVAL_MS 2000
#define STATUS_PRINT_INTERVAL_MS 1000
#define AUTO_RECONNECT_ENABLED true

struct imu_slot_t
{
  i2c_target_t target;
  stmdev_ctx_t ctx;
  bool present;
  bool eligibleForRetry;    // true only if this slot connected at the initial boot probe
  uint16_t disconnectCount; // counts present->not-present transitions, not retry attempts
  float lastAccelZmg;
  uint32_t nextRetryMs; // millis() timestamp of the next detect/re-detect attempt
};

static i2c_target_t imuTargets[NUM_IMUS];
static imu_slot_t imus[NUM_IMUS];

// 2 IMUs per bus: IMU 2n is at 0x6A and IMU 2n+1 at 0x6B on bus n.
static uint8_t sensorBus(uint8_t i) { return i / 2; }
static uint8_t sensorAddr(uint8_t i) { return (i % 2) ? LSM6DSV_ADDR_H : LSM6DSV_ADDR_L; }

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  i2c_target_t *target = (i2c_target_t *)handle;
  TwoWire *wire = i2cBusWire(target->bus);

  wire->beginTransmission(target->addr);
  wire->write(reg);
  wire->write(bufp, len);
  return wire->endTransmission();
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  i2c_target_t *target = (i2c_target_t *)handle;
  TwoWire *wire = i2cBusWire(target->bus);

  wire->beginTransmission(target->addr);
  wire->write(reg);
  if (wire->endTransmission(false) != 0)
  {
    return -1;
  }

  uint16_t received = wire->requestFrom(target->addr, (size_t)len);
  if (received != len)
  {
    return -1;
  }

  for (uint16_t i = 0; i < len; i++)
  {
    bufp[i] = wire->read();
  }
  return 0;
}

static void platform_delay(uint32_t ms) { delay(ms); }

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
  if (!i2cBusClaimIfIdle(sensorBus(i)))
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

  // Hardware sensor fusion: the SFLP engine computes a game rotation vector
  // (quaternion, minus the w component -- see imuSystemPoll()'s FIFO drain)
  // on-chip and delivers it through the FIFO, independent of the accel/gyro
  // data-ready polling above.
  lsm6dsv_sflp_game_rotation_set(&imus[i].ctx, PROPERTY_ENABLE);
  lsm6dsv_sflp_data_rate_set(&imus[i].ctx, LSM6DSV_SFLP_120Hz);
  lsm6dsv_fifo_sflp_raw_t sflpBatch = {0};
  sflpBatch.game_rotation = 1;
  lsm6dsv_fifo_sflp_batch_set(&imus[i].ctx, sflpBatch);
  lsm6dsv_fifo_mode_set(&imus[i].ctx, LSM6DSV_STREAM_MODE);

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

static uint8_t presentCountAtBoot = 0;

void imuSystemInit()
{
  // Buses are claimed lazily, one at a time, the first time initImu() finds
  // each one idle -- see i2cBusClaimIfIdle(). No eager wire->begin() loop here.

  for (uint8_t i = 0; i < NUM_IMUS; i++)
  {
    imuTargets[i].bus = sensorBus(i);
    imuTargets[i].addr = sensorAddr(i);

    imus[i].target = imuTargets[i];
    imus[i].ctx.write_reg = platform_write;
    imus[i].ctx.read_reg = platform_read;
    imus[i].ctx.mdelay = platform_delay;
    imus[i].ctx.handle = &imuTargets[i];
    imus[i].present = false;
    imus[i].eligibleForRetry = false;
    imus[i].disconnectCount = 0;
    imus[i].lastAccelZmg = NAN;
    imus[i].nextRetryMs = 0;
  }

  platform_delay(10); // sensor boot time

  for (uint8_t i = 0; i < NUM_IMUS; i++)
  {
    imu_init_result_t result = initImu(i);
    imus[i].present = (result == IMU_INIT_OK);
    // Only sensors that respond at boot ever get retried again -- a slot
    // that fails here is excluded from loop()'s retry logic for the rest
    // of the run, even if something is plugged into it later.
    imus[i].eligibleForRetry = imus[i].present;
    imus[i].nextRetryMs = millis() + OFFLINE_RETRY_INTERVAL_MS;
    if (imus[i].present)
    {
      presentCountAtBoot++;
    }

    Serial.print("IMU");
    Serial.print(i);
    Serial.println(initResultMessage(result));
  }
}

uint8_t imuPresentCount() { return presentCountAtBoot; }

// Builds and prints every eligible IMU's status as fixed-width fields on one
// line, in a fixed order (boot-time eligibility never changes at runtime),
// so the printed line always has the same shape -- easy to eyeball a single
// column changing over time in a terminal instead of scanning N lines.
static void printStatusTable()
{
  char lineBuf[NUM_IMUS * 40 + 1] = {0}; // zeroed: if no IMU is eligible, offset stays 0 and
                                         // this must still be a valid (empty) C string
  size_t offset = 0;
  bool first = true;

  for (uint8_t i = 0; i < NUM_IMUS; i++)
  {
    if (!imus[i].eligibleForRetry)
    {
      continue; // never connected at boot -- excluded from the ongoing view
    }

    char zBuf[12];
    if (isnan(imus[i].lastAccelZmg))
    {
      snprintf(zBuf, sizeof(zBuf), "%7s", "n/a");
    }
    else
    {
      // Not %7.1f via snprintf: the Feather toolchain's libc has no float
      // printf support by default and silently emits nothing for %f, rather
      // than erroring -- dtostrf() is the portable Arduino-core way to format
      // a float without depending on that (and exists on ESP32 too).
      dtostrf(imus[i].lastAccelZmg, 7, 1, zBuf);
    }

    int written = snprintf(lineBuf + offset, sizeof(lineBuf) - offset,
                           "%sIMU%-2u %-7s drops=%3u z=%smg", first ? "" : " | ", i,
                           imus[i].present ? "ONLINE" : "OFFLINE", imus[i].disconnectCount, zBuf);
    if (written > 0)
    {
      offset += written;
    }
    first = false;
  }

  Serial.println(first ? "no IMU detected" : lineBuf);
}

void imuMaybePrintStatusTable(uint32_t now)
{
  static uint32_t nextStatusPrintMs = 0;
  if ((int32_t)(now - nextStatusPrintMs) >= 0)
  {
    nextStatusPrintMs = now + STATUS_PRINT_INTERVAL_MS;
    printStatusTable();
  }
}

static bool cachedAnyOffline = false;
static bool cachedAnyEverDisconnected = false;

void imuSystemPoll(uint32_t now)
{
  for (uint8_t i = 0; i < NUM_IMUS; i++)
  {
    if (!imus[i].eligibleForRetry)
    {
      // Never responded at boot -- permanently excluded, not just offline.
      continue;
    }

    if (!imus[i].present)
    {
#if AUTO_RECONNECT_ENABLED
      // Periodically re-probe a sensor that dropped after previously working.
      if ((int32_t)(now - imus[i].nextRetryMs) >= 0)
      {
        imus[i].nextRetryMs = now + OFFLINE_RETRY_INTERVAL_MS;
        imu_init_result_t result = initImu(i);
        imus[i].present = (result == IMU_INIT_OK);

        Serial.print("IMU");
        Serial.print(i);
        Serial.println(imus[i].present ? ": back online" : initResultMessage(result));
      }
#endif
      continue;
    }

    lsm6dsv_data_ready_t drdy;
    if (lsm6dsv_flag_data_ready_get(&imus[i].ctx, &drdy) != 0)
    {
      // Lost communication with a previously-working sensor. The failed
      // transaction may have left this bus electrically wedged (see
      // i2cBusRecover()) -- clear that now rather than waiting for the next
      // scheduled retry to discover the same wedge all over again.
      i2cBusRecover(sensorBus(i));

      imus[i].present = false;
      imus[i].disconnectCount++;
      imus[i].nextRetryMs = now + OFFLINE_RETRY_INTERVAL_MS;

      Serial.print("IMU");
      Serial.print(i);
      Serial.println(": OFFLINE (lost communication)");
      continue;
    }

    if (drdy.drdy_xl || drdy.drdy_gy)
    {
      int16_t data_raw[3];

      if (drdy.drdy_xl)
      {
        lsm6dsv_acceleration_raw_get(&imus[i].ctx, data_raw);
        imus[i].lastAccelZmg = lsm6dsv_from_fs4_to_mg(data_raw[2]);

        imuOutputAccel(i, (int16_t)lroundf(lsm6dsv_from_fs4_to_mg(data_raw[0])),
                       (int16_t)lroundf(lsm6dsv_from_fs4_to_mg(data_raw[1])),
                       (int16_t)lroundf(imus[i].lastAccelZmg));
      }

      if (drdy.drdy_gy)
      {
        lsm6dsv_angular_rate_raw_get(&imus[i].ctx, data_raw);

        // 0.1 dps/LSB: mdps/100 rounds to the nearest tenth of a degree/sec.
        imuOutputGyro(i, (int16_t)lroundf(lsm6dsv_from_fs2000_to_mdps(data_raw[0]) / 100.0f),
                      (int16_t)lroundf(lsm6dsv_from_fs2000_to_mdps(data_raw[1]) / 100.0f),
                      (int16_t)lroundf(lsm6dsv_from_fs2000_to_mdps(data_raw[2]) / 100.0f));
      }
    }

    // Drain the SFLP quaternion FIFO -- separate from the drdy-based
    // accel/gyro path above, since SFLP data only ever arrives via FIFO.
    lsm6dsv_fifo_status_t fifoStatus;
    if (lsm6dsv_fifo_status_get(&imus[i].ctx, &fifoStatus) == 0)
    {
      while (fifoStatus.fifo_level > 0)
      {
        lsm6dsv_fifo_out_raw_t rec;
        if (lsm6dsv_fifo_out_raw_get(&imus[i].ctx, &rec) != 0)
        {
          break;
        }
        if (rec.tag == LSM6DSV_SFLP_GAME_ROTATION_VECTOR_TAG)
        {
          // rec.data holds qx, qy, qz as 3 raw half-precision floats
          // (little-endian) straight from the sensor -- forwarded as-is,
          // no conversion. qw isn't transmitted; a unit-quaternion receiver
          // reconstructs it with qw = sqrt(max(0, 1 - qx^2 - qy^2 - qz^2)).
          uint16_t qx = rec.data[0] | (rec.data[1] << 8);
          uint16_t qy = rec.data[2] | (rec.data[3] << 8);
          uint16_t qz = rec.data[4] | (rec.data[5] << 8);
          imuOutputQuat(i, qx, qy, qz);
        }
        fifoStatus.fifo_level--;
      }
    }
  }

  cachedAnyOffline = false;
  cachedAnyEverDisconnected = false;
  for (uint8_t i = 0; i < NUM_IMUS; i++)
  {
    if (!imus[i].eligibleForRetry)
    {
      continue;
    }
    if (!imus[i].present)
    {
      cachedAnyOffline = true;
    }
    if (imus[i].disconnectCount > 0)
    {
      cachedAnyEverDisconnected = true;
    }
  }
}

bool imuAnyOffline() { return cachedAnyOffline; }

bool imuAnyEverDisconnected() { return cachedAnyEverDisconnected; }
