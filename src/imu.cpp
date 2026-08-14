#include <Arduino.h>
#include <Wire.h>
#include <avr/dtostrf.h>    // dtostrf() -- this core's own float-to-string emulation,
#include <math.h>           // NAN, isnan(), lroundf()
#include <stdio.h>          // snprintf()
#include <wiring_private.h> // pinPeripheral()
                            // since its libc snprintf has no %f support
#include "can_bus.h"
#include "imu.h"

extern "C"
{
#include "lsm6dsv_reg.h"
}

// SDO/SA0 tied low -> 0x6A, tied high -> 0x6B (7-bit I2C address)
#define LSM6DSV_ADDR_L (LSM6DSV_I2C_ADD_L >> 1)
#define LSM6DSV_ADDR_H (LSM6DSV_I2C_ADD_H >> 1)

#define NUM_BUSES 5

// Bus 0 uses the board's default STEMMA QT Wire (SERCOM2, D21/D22).
// Buses 1-4 repurpose SERCOM3/4/0/5, whose pads land on otherwise-free
// header pins on this board (verified against the Feather M4 CAN schematic).
// None of these pins carry pull-ups on this board -- pull-ups come from
// whichever IMU board is plugged into a given bus, not something to add
// separately here.
static TwoWire busWire1(&sercom3, 12, 13); // D12 (SDA), D13 (SCL) # tested works with 2 imus
static TwoWire busWire2(&sercom4, 16, 17); // A2  (SDA), A3  (SCL) # tested works with 2 imus
static TwoWire busWire3(&sercom0, 18, 15); // A4  (SDA), A1  (SCL) # tested works with 2 imus.
static TwoWire busWire4(
    &sercom5, 1,
    0); // D1 TX (SDA), D0 Rx (SCL) # tested works with 2 imus. -- takes over Serial1's pins

struct bus_desc_t
{
  TwoWire *wire;
  uint8_t pinSDA;
  uint8_t pinSCL;
  EPioType muxType; // mux function TwoWire::begin() needs forced onto pinSDA/pinSCL
  bool claimed;     // true once claimBusIfIdle() has called wire->begin() on this bus
};

static bus_desc_t buses[NUM_BUSES] = {
    {&Wire, 21, 22, PIO_SERCOM, false},         // default mux already correct
    {&busWire1, 12, 13, PIO_SERCOM, false},     // SERCOM3 is the primary mux on D12/D13
    {&busWire2, 16, 17, PIO_SERCOM_ALT, false}, // SERCOM4 is the alt mux on A2/A3
    {&busWire3, 18, 15, PIO_SERCOM_ALT, false}, // SERCOM0 is the alt mux on A4/A1
    {&busWire4, 1, 0, PIO_SERCOM, false},       // SERCOM5 is the primary mux on D0/D1
};

struct i2c_target_t
{
  TwoWire *wire;
  uint8_t addr;
};

#define OFFLINE_RETRY_INTERVAL_MS 2000
#define STATUS_PRINT_INTERVAL_MS 1000
#define AUTO_RECONNECT_ENABLED true
// LSM6DSV supports up to 1000000 (Fast-mode+); 400000 (Fast-mode) is a more
// forgiving next step up from 100000 given the pull-up/wiring situation.
#define I2C_CLOCK_HZ 400000

struct imu_slot_t
{
  i2c_target_t target;
  stmdev_ctx_t ctx;
  bool present;
  bool eligibleForRetry;    // true only if this slot connected at the initial boot probe
  uint16_t disconnectCount; // counts present->not-present transitions, not retry attempts
  float lastAccelZmg;
  uint32_t nextRetryMs; // millis() timestamp of the next detect/re-detect attempt
  uint8_t accelCanSeq;  // per-data-type CAN sequence counters, wrap at 256
  uint8_t gyroCanSeq;
  uint8_t quatCanSeq;
};

static i2c_target_t imuTargets[NUM_IMUS];
static imu_slot_t imus[NUM_IMUS];

// bus index and I2C address for each of the 10 IMUs (2 per bus)
static const uint8_t SENSOR_BUS[NUM_IMUS] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4};
static const uint8_t SENSOR_ADDR[NUM_IMUS] = {
    LSM6DSV_ADDR_L, LSM6DSV_ADDR_H, LSM6DSV_ADDR_L, LSM6DSV_ADDR_H, LSM6DSV_ADDR_L,
    LSM6DSV_ADDR_H, LSM6DSV_ADDR_L, LSM6DSV_ADDR_H, LSM6DSV_ADDR_L, LSM6DSV_ADDR_H,
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

static void platform_delay(uint32_t ms) { delay(ms); }

// A bus with no pull-ups (nothing wired to it yet, or a dead connection)
// never reaches a valid idle-high state. The vendor SERCOM I2C driver's
// transmission-start wait loop has no timeout and spins forever in that
// case, so idle must be confirmed *before* any blocking Wire call --
// otherwise one unwired bus permanently freezes the whole sketch.
//
// A genuinely unconnected pin is left floating by a plain INPUT, and a
// floating pin's read is undefined -- ambient noise/coupling can make it
// read HIGH by chance, which would pass the idle check below and let a
// bus with nothing attached get claimed anyway, hanging forever on the
// first real I2C transaction. INPUT_PULLDOWN removes that ambiguity: an
// unconnected pin now reads a deterministic LOW (failing the check, as it
// should), while a real external pull-up (a few kOhm) still overpowers
// this weak internal pull-down (tens of kOhm) by a wide margin, so a
// properly wired bus still reads solidly HIGH and isn't affected.
//
// Once a bus is claimed (wire->begin() called), this never touches
// pinMode() on its pins again. Empirically, doing a pinMode(INPUT) ->
// pinPeripheral(restore) round-trip on an already-begin()'d bus breaks I2C
// communication on it -- even on the very first transaction afterward, on
// a bus/device that otherwise works fine (observed on bus0/SERCOM2 with a
// real LSM6DSV attached). So the idle check only ever runs while a bus's
// pins are still unclaimed plain GPIO (nothing active to disrupt); past
// that point, retries rely on the I2C transaction's own error return to
// detect a dead/removed sensor instead.
static bool claimBusIfIdle(bus_desc_t &bus)
{
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
    return false; // still unclaimed plain GPIO -- nothing to restore
  }

  bus.wire->begin();
  // TwoWire::begin() re-applies each pin's *default* mux, which is wrong
  // for pins whose I2C function is not their default (A1-A4, D12, D13) --
  // force the correct SERCOM mux back onto them here. This is the pins'
  // first-ever claim (unlike the old busIsIdle(), which repeated this on
  // every retry), so there's no already-active peripheral to disrupt.
  pinPeripheral(bus.pinSDA, bus.muxType);
  pinPeripheral(bus.pinSCL, bus.muxType);
  bus.wire->setClock(I2C_CLOCK_HZ);
  bus.claimed = true;
  return true;
}

// Standard I2C bus-recovery procedure (NXP UM10204 S3.1.16): a slave that
// was interrupted mid-transaction (e.g. platform_read()/platform_write()
// aborting on an error without ever sending a STOP) can be left holding
// SDA low indefinitely. A plain STOP can't fix that -- STOP needs SDA free
// to rise, which it isn't while a slave is actively driving it low. Only a
// master-generated clock train gives the slave a chance to finish
// whatever bit it's stuck on and let go; that's what this does, followed
// by a manufactured STOP once SDA is (or already was) free.
//
// This is deliberately unlike claimBusIfIdle()'s pinMode() round-trip,
// which broke I2C when done on a bus whose SERCOM peripheral was still
// enabled (see that function's comment). Here wire->end() fully disables
// the peripheral *before* any pin is touched, so there's no live SERCOM
// state for the remux to clash with -- the pins are inert GPIO for the
// whole bit-banged sequence, exactly as during the original (working)
// claim.
static void recoverBus(bus_desc_t &bus)
{
  bus.wire->end();

  pinMode(bus.pinSCL, OUTPUT);
  pinMode(bus.pinSDA, INPUT); // read-only: let the slave/pull-up drive it

  for (uint8_t i = 0; i < 9 && digitalRead(bus.pinSDA) == LOW; i++)
  {
    digitalWrite(bus.pinSCL, LOW);
    delayMicroseconds(5);
    digitalWrite(bus.pinSCL, HIGH);
    delayMicroseconds(5);
  }

  // Manufacture a STOP condition: SDA rising while SCL is high.
  pinMode(bus.pinSDA, OUTPUT);
  digitalWrite(bus.pinSDA, LOW);
  delayMicroseconds(5);
  digitalWrite(bus.pinSCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(bus.pinSDA, HIGH);
  delayMicroseconds(5);

  // Hand the pins back to the SERCOM peripheral and re-init the master --
  // same begin()-then-remux order claimBusIfIdle() uses, since begin()
  // re-applies each pin's *default* (non-I2C) mux.
  bus.wire->begin();
  pinPeripheral(bus.pinSDA, bus.muxType);
  pinPeripheral(bus.pinSCL, bus.muxType);
  bus.wire->setClock(I2C_CLOCK_HZ);
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
  if (!claimBusIfIdle(buses[SENSOR_BUS[i]]))
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
  // each one idle -- see claimBusIfIdle(). No eager wire->begin() loop here.

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
    imus[i].eligibleForRetry = false;
    imus[i].disconnectCount = 0;
    imus[i].lastAccelZmg = NAN;
    imus[i].nextRetryMs = 0;
    imus[i].accelCanSeq = 0;
    imus[i].gyroCanSeq = 0;
    imus[i].quatCanSeq = 0;
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
      // Not %7.1f via snprintf: this toolchain's libc has no float printf
      // support by default and silently emits nothing for %f, rather than
      // erroring -- dtostrf() is the portable Arduino-core way to format a
      // float without depending on that.
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
      // recoverBus()) -- clear that now rather than waiting for the next
      // scheduled retry to discover the same wedge all over again.
      recoverBus(buses[SENSOR_BUS[i]]);

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

        sendCanFrame(CAN_ID_ACCEL_BASE + i, (int16_t)lroundf(lsm6dsv_from_fs4_to_mg(data_raw[0])),
                     (int16_t)lroundf(lsm6dsv_from_fs4_to_mg(data_raw[1])),
                     (int16_t)lroundf(imus[i].lastAccelZmg), imus[i].accelCanSeq);
      }

      if (drdy.drdy_gy)
      {
        lsm6dsv_angular_rate_raw_get(&imus[i].ctx, data_raw);

        // 0.1 dps/LSB: mdps/100 rounds to the nearest tenth of a degree/sec.
        sendCanFrame(CAN_ID_GYRO_BASE + i,
                     (int16_t)lroundf(lsm6dsv_from_fs2000_to_mdps(data_raw[0]) / 100.0f),
                     (int16_t)lroundf(lsm6dsv_from_fs2000_to_mdps(data_raw[1]) / 100.0f),
                     (int16_t)lroundf(lsm6dsv_from_fs2000_to_mdps(data_raw[2]) / 100.0f),
                     imus[i].gyroCanSeq);
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
          sendCanFrame(CAN_ID_QUAT_BASE + i, qx, qy, qz, imus[i].quatCanSeq);
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
