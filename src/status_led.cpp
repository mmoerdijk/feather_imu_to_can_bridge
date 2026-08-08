#include <Adafruit_NeoPixel.h>
#include "status_led.h"

// Onboard NeoPixel (PIN_NEOPIXEL/PIN_NEOPIXEL_POWER from the board variant) --
// power rail is already switched on for us by the core's initVariant().
static Adafruit_NeoPixel statusLed(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

void ledInit()
{
  statusLed.begin();
  statusLed.setBrightness(20);
  statusLed.setPixelColor(0, statusLed.Color(0, 0, 255)); // blue: detecting
  statusLed.show();
}

void ledFlashDetectedCount(uint8_t count)
{
  // Report the detection count as white flashes, then settle to green --
  // the LED's steady "everything's fine" run color.
  for (uint8_t i = 0; i < count; i++)
  {
    statusLed.setPixelColor(0, statusLed.Color(255, 255, 255));
    statusLed.show();
    delay(400);
    statusLed.setPixelColor(0, 0);
    statusLed.show();
    delay(400);
  }
  statusLed.setPixelColor(0, statusLed.Color(0, 255, 0));
  statusLed.show();
}

void ledUpdateRunState(bool anyOffline, bool anyEverDisconnected)
{
  // Worst-current-state wins: red (something's offline right now) beats
  // orange (recovered from a past drop) beats green (never had an issue).
  uint32_t ledColor = anyOffline ? statusLed.Color(255, 0, 0)
                      : anyEverDisconnected ? statusLed.Color(255, 80, 0)
                                            : statusLed.Color(0, 255, 0);
  static uint32_t lastLedColor = 0;
  if (ledColor != lastLedColor)
  {
    lastLedColor = ledColor;
    statusLed.setPixelColor(0, ledColor);
    statusLed.show();
  }
}
