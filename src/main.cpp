#include <Arduino.h>
#include "can_bus.h"
#include "imu.h"
#include "status_led.h"

void setup()
{
  ledInit();

  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }

  canBusInit();
  imuSystemInit();
  ledFlashDetectedCount(imuPresentCount());
}

void loop()
{
  uint32_t now = millis();

  canBusPoll();
  imuSystemPoll(now);
  ledUpdateRunState(imuAnyOffline(), imuAnyEverDisconnected());
  imuMaybePrintStatusTable(now);
}
