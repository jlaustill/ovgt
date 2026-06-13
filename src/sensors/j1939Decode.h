#ifndef j1939Decode_h
#define j1939Decode_h

#include <stdint.h>

// Extract the PGN from a 29-bit J1939 CAN identifier.
// For PDU2 (PF >= 240) the PGN includes the PS byte; for PDU1 it does not.
uint32_t pgnFromCanId(uint32_t canId);

// ETC1 (PGN 61442) byte 1 bits 3-4 -> SPN 573 Torque Converter Lockup (0-3).
uint8_t decodeTorqueConverterLockup(const uint8_t *buf);

// EEC2 (PGN 61443) byte 2 -> SPN 91 Accelerator Pedal Position 1 (0.4 %/bit).
uint8_t decodeAcceleratorPedalPercent(const uint8_t *buf);

// EEC2 (PGN 61443) byte 3 -> SPN 92 Engine Percent Load (1 %/bit, 0-250).
uint8_t decodeEngineLoadPercent(const uint8_t *buf);

#endif
