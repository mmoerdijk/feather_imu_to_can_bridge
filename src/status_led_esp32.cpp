#if defined(ARDUINO_ARCH_ESP32)

#include "status_led.h"
#include <Arduino.h>

// The S2 Mini has no NeoPixel, only a single-color (blue) LED on GPIO15,
// active high -- so the Feather's colors become blink patterns instead:
//
//   Feather color          S2 Mini LED
//   blue (detecting)       solid on
//   white flashes (count)  slow flashes, one per IMU found
//   green (all fine)       solid on
//   orange (recovered)     on, with a short off-blip once per second
//   red (offline now)      blinking 2x per second
//   red blink (none found) rapid flicker
#define STATUS_LED_PIN LED_BUILTIN

static void ledWrite(bool on) { digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW); }

void ledInit()
{
  pinMode(STATUS_LED_PIN, OUTPUT);
  ledWrite(true); // detecting
}

void ledFlashDetectedCount(uint8_t count)
{
  if (count == 0)
  {
    // Nothing found at all -- ledUpdateRunState() takes over with a rapid
    // flicker from the very first loop() iteration.
    return;
  }

  // Off first, so the first flash is distinguishable from the solid-on
  // detecting state, then one flash per IMU, then settle to solid on.
  ledWrite(false);
  delay(800);
  for (uint8_t i = 0; i < count; i++)
  {
    ledWrite(true);
    delay(400);
    ledWrite(false);
    delay(400);
  }
  ledWrite(true);
}

void ledUpdateRunState(bool noneDetected, bool anyOffline, bool anyEverDisconnected)
{
  // Worst-current-state wins, same priority as the Feather's colors.
  uint16_t periodMs;
  uint16_t onMs;
  if (noneDetected)
  {
    periodMs = 100;
    onMs = 50;
  }
  else if (anyOffline)
  {
    periodMs = 500;
    onMs = 250;
  }
  else if (anyEverDisconnected)
  {
    periodMs = 1000;
    onMs = 900;
  }
  else
  {
    periodMs = 1;
    onMs = 1; // solid on
  }

  bool on = (millis() % periodMs) < onMs;
  static bool lastOn = true; // ledFlashDetectedCount()/ledInit() leave it on
  if (on != lastOn)
  {
    lastOn = on;
    ledWrite(on);
  }
}

#endif // ARDUINO_ARCH_ESP32
