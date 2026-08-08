#pragma once

#include <stdint.h>

#define CAN_ID_ACCEL_BASE 0x100
#define CAN_ID_GYRO_BASE 0x110
#define CAN_ID_QUAT_BASE 0x120

// Starts the onboard CAN0 transceiver at CAN_BITRATE. Call once from setup().
void canBusInit();

// Prints any frame this board receives -- only produces output when
// CAN_LOOPBACK_VERIFY_ENABLED is true (canBusInit() also enables loopback
// in that case, so this reflects every frame just sent). Call once per
// loop() iteration; a no-op otherwise.
void canBusPoll();

// Packs 3 raw 16-bit values (already in each frame's wire encoding -- mg
// fixed-point, 0.1dps fixed-point, or a passthrough FP16 bit pattern,
// depending on caller) plus a sequence counter into one 8-byte CAN frame:
// [v0 LE][v1 LE][v2 LE][seq][reserved=0]. No-ops if CAN.begin() never
// succeeded, so a CAN wiring/hardware problem doesn't affect IMU polling.
void sendCanFrame(uint32_t id, uint16_t v0, uint16_t v1, uint16_t v2, uint8_t &seq);
