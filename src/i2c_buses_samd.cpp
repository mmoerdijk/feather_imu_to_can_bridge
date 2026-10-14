#if defined(ARDUINO_ARCH_SAMD)

#include "i2c_buses.h"
#include <Arduino.h>
#include <wiring_private.h> // pinPeripheral()

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
  bool claimed;     // true once i2cBusClaimIfIdle() has called wire->begin() on this bus
};

static bus_desc_t buses[NUM_BUSES] = {
    {&Wire, 21, 22, PIO_SERCOM, false},         // default mux already correct
    {&busWire1, 12, 13, PIO_SERCOM, false},     // SERCOM3 is the primary mux on D12/D13
    {&busWire2, 16, 17, PIO_SERCOM_ALT, false}, // SERCOM4 is the alt mux on A2/A3
    {&busWire3, 18, 15, PIO_SERCOM_ALT, false}, // SERCOM0 is the alt mux on A4/A1
    {&busWire4, 1, 0, PIO_SERCOM, false},       // SERCOM5 is the primary mux on D0/D1
};

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
bool i2cBusClaimIfIdle(uint8_t busIndex)
{
  bus_desc_t &bus = buses[busIndex];
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

TwoWire *i2cBusWire(uint8_t busIndex) { return buses[busIndex].wire; }

// A slave that was interrupted mid-transaction (e.g. platform_read()/
// platform_write() aborting on an error without ever sending a STOP) can be
// left holding SDA low indefinitely. A plain STOP can't fix that -- STOP
// needs SDA free to rise, which it isn't while a slave is actively driving
// it low. Only a master-generated clock train gives the slave a chance to
// finish whatever bit it's stuck on and let go; that's what this does,
// followed by a manufactured STOP once SDA is (or already was) free.
//
// This is deliberately unlike i2cBusClaimIfIdle()'s pinMode() round-trip,
// which broke I2C when done on a bus whose SERCOM peripheral was still
// enabled (see that function's comment). Here wire->end() fully disables
// the peripheral *before* any pin is touched, so there's no live SERCOM
// state for the remux to clash with -- the pins are inert GPIO for the
// whole bit-banged sequence, exactly as during the original (working)
// claim.
void i2cBusRecover(uint8_t busIndex)
{
  bus_desc_t &bus = buses[busIndex];
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
  // same begin()-then-remux order i2cBusClaimIfIdle() uses, since begin()
  // re-applies each pin's *default* (non-I2C) mux.
  bus.wire->begin();
  pinPeripheral(bus.pinSDA, bus.muxType);
  pinPeripheral(bus.pinSCL, bus.muxType);
  bus.wire->setClock(I2C_CLOCK_HZ);
}

#endif // ARDUINO_ARCH_SAMD
