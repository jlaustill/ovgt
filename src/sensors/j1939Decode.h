#ifndef j1939Decode_h
#define j1939Decode_h

#include <stdint.h>

// Extract the PGN from a 29-bit J1939 CAN identifier.
// For PDU2 (PF >= 240) the PGN includes the PS byte; for PDU1 it does not.
uint32_t pgnFromCanId(uint32_t canId);

// Extract the source address (low byte) from a 29-bit J1939 CAN identifier.
uint8_t saFromCanId(uint32_t canId);

// ETC1 (PGN 61442) byte 1 bits 3-4 -> SPN 573 Torque Converter Lockup (0-3).
uint8_t decodeTorqueConverterLockup(const uint8_t *buf);

// EEC2 (PGN 61443) byte 2 -> SPN 91 Accelerator Pedal Position 1 (0.4 %/bit).
uint8_t decodeAcceleratorPedalPercent(const uint8_t *buf);

// EEC2 (PGN 61443) byte 3 -> SPN 92 Engine Percent Load (1 %/bit, 0-250).
uint8_t decodeEngineLoadPercent(const uint8_t *buf);

// EEC1 (PGN 61444) bytes 3-4 LE -> SPN 190 Engine Speed (0.125 rpm/bit).
uint16_t decodeEngineRpm(const uint8_t *buf);
// EEC1 byte 1 -> SPN 512 Driver's Demand Percent Torque (1 %/bit, offset -125).
int8_t decodeDriverDemandTorquePct(const uint8_t *buf);
// EEC1 byte 2 -> SPN 513 Actual Engine Percent Torque (1 %/bit, offset -125).
int8_t decodeActualTorquePct(const uint8_t *buf);

// IC1 (PGN 65270) byte 2 -> SPN 105 Intake Manifold 1 Temp (1 C/bit, offset -40).
int16_t decodeIntakeAirTempC(const uint8_t *buf);
// IC1 byte 1 -> SPN 102 Boost Pressure (2 kPa/bit).
uint16_t decodeBoostKpa(const uint8_t *buf);
// AMB (PGN 65269) byte 0 -> SPN 108 Barometric/pre-turbo pressure (0.5 kPa/bit).
uint16_t decodePreTurboKpa(const uint8_t *buf);

// ET1 (PGN 65262) byte 0 -> SPN 110 Coolant Temp (1 C/bit, offset -40).
int16_t decodeCoolantTempC(const uint8_t *buf);
// ET1 bytes 2-3 LE -> SPN 175 Engine Oil Temp 1 (0.03125 C/bit, offset -273).
int16_t decodeOilTempC(const uint8_t *buf);
// EFL/P1 (PGN 65263) byte 3 -> SPN 100 Engine Oil Pressure (4 kPa/bit).
uint16_t decodeOilPressureKpa(const uint8_t *buf);

#endif
