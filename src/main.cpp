#include "imu.h"
#include "imu_output.h"
#include "status_led.h"
#include <Arduino.h>

void setup()
{
  ledInit();

  Serial.begin(115200);
  // Bounded wait only: on standalone power (no USB host attached) Serial
  // never becomes ready, and an unbounded while(!Serial) would hang here
  // forever -- exactly what caused the cold-start hang on the blue LED.
  uint32_t serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart) < 2000)
  {
    delay(10);
  }

  imuOutputInit();
  imuSystemInit();
  ledFlashDetectedCount(imuPresentCount());
}

void loop()
{
  uint32_t now = millis();

  imuOutputPoll();
  imuSystemPoll(now);
  ledUpdateRunState(imuPresentCount() == 0, imuAnyOffline(), imuAnyEverDisconnected());
  imuMaybePrintStatusTable(now);
}
