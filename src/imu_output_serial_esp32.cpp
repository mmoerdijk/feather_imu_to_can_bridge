#if defined(ARDUINO_ARCH_ESP32)

#include "imu_output.h"
#include <Arduino.h>
#include <math.h>

// ESP32-S2 test build: no CAN -- every reading is printed to Serial (the S2
// Mini's native USB port) as one text line, e.g.
//
//   IMU0 acc   -12     3  1002 mg
//   IMU0 gyr   0.4  -1.2   0.0 dps
//   IMU0 quat w=0.9998 x=0.0123 y=-0.0051 z=0.0002
//
// These interleave with the once-per-second status line from imu.cpp.

void imuOutputInit() { Serial.println("Output: Serial (no CAN in the ESP32 test build)"); }

void imuOutputPoll() {}

// Skip printing while no terminal has the port open: USB CDC writes then
// only fill the TX buffer and wait out its timeout, which would throttle
// IMU polling for nothing.
static bool terminalOpen() { return (bool)Serial; }

void imuOutputAccel(uint8_t imu, int16_t xMg, int16_t yMg, int16_t zMg)
{
  if (terminalOpen())
  {
    Serial.printf("IMU%u acc %5d %5d %5d mg\n", imu, xMg, yMg, zMg);
  }
}

void imuOutputGyro(uint8_t imu, int16_t xDdps, int16_t yDdps, int16_t zDdps)
{
  if (terminalOpen())
  {
    Serial.printf("IMU%u gyr %6.1f %6.1f %6.1f dps\n", imu, xDdps / 10.0f, yDdps / 10.0f,
                  zDdps / 10.0f);
  }
}

// IEEE-754 half precision -> float.
static float halfToFloat(uint16_t h)
{
  int sign = (h & 0x8000) ? -1 : 1;
  int exponent = (h >> 10) & 0x1F;
  int mantissa = h & 0x3FF;
  if (exponent == 0)
  {
    return sign * ldexpf((float)mantissa, -24); // zero / subnormal
  }
  if (exponent == 31)
  {
    return mantissa ? NAN : sign * INFINITY;
  }
  return sign * ldexpf((float)(mantissa | 0x400), exponent - 25);
}

void imuOutputQuat(uint8_t imu, uint16_t qx, uint16_t qy, uint16_t qz)
{
  if (terminalOpen())
  {
    float x = halfToFloat(qx);
    float y = halfToFloat(qy);
    float z = halfToFloat(qz);
    float w = sqrtf(fmaxf(0.0f, 1.0f - x * x - y * y - z * z));
    Serial.printf("IMU%u quat w=%.4f x=%.4f y=%.4f z=%.4f\n", imu, w, x, y, z);
  }
}

#endif // ARDUINO_ARCH_ESP32
