#include "can_bus.h"
#include <Arduino.h>
#include <CANSAME5x.h>

// Onboard CAN transceiver (CANSAME5x wraps the SAME51's built-in CAN0, fixed
// to PIN_CAN_TX/PIN_CAN_RX -- separate pins from the I2C buses used for the
// IMUs).
static CANSAME5x CAN;
#define CAN_BITRATE 1000000
static bool canReady = false;

// Loops every transmitted CAN frame back to this board's own receiver (still
// also drives the physical bus/TX pin), so sending can be checked over
// Serial without a CAN-to-USB adapter. Off by default -- flip to true and
// reflash to verify CAN traffic again.
#define CAN_LOOPBACK_VERIFY_ENABLED false

void canBusInit()
{
  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, LOW); // turn off standby
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, HIGH); // turn on booster
  canReady = CAN.begin(CAN_BITRATE);
  Serial.println(canReady ? "CAN: started"
                          : "CAN: failed to start -- IMU data won't be sent over CAN");
#if CAN_LOOPBACK_VERIFY_ENABLED
  if (canReady)
  {
    CAN.loopback();
  }
#endif
}

void canBusPoll()
{
#if CAN_LOOPBACK_VERIFY_ENABLED
  // Prints every CAN frame this board receives -- with CAN.loopback() in
  // canBusInit(), that's every frame it just sent.
  int canPacketSize = CAN.parsePacket();
  if (canPacketSize > 0)
  {
    Serial.print("CAN rx id=0x");
    Serial.print(CAN.packetId(), HEX);
    Serial.print(" data=");
    while (CAN.available())
    {
      int b = CAN.read();
      if (b < 0x10)
      {
        Serial.print('0');
      }
      Serial.print(b, HEX);
      Serial.print(' ');
    }
    Serial.println();
  }
#endif
}

void sendCanFrame(uint32_t id, uint16_t v0, uint16_t v1, uint16_t v2, uint8_t &seq)
{
  if (!canReady)
  {
    return;
  }

  uint8_t payload[8] = {
      (uint8_t)(v0 & 0xFF),
      (uint8_t)(v0 >> 8),
      (uint8_t)(v1 & 0xFF),
      (uint8_t)(v1 >> 8),
      (uint8_t)(v2 & 0xFF),
      (uint8_t)(v2 >> 8),
      seq++,
      0,
  };
  CAN.beginPacket(id);
  CAN.write(payload, sizeof(payload));
  CAN.endPacket();
}
