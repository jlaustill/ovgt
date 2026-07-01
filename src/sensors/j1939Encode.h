#ifndef j1939Encode_h
#define j1939Encode_h

#include <stdint.h>

// Encode physical values into J1939 raw 2-byte (little-endian) field values,
// clamped to the maximum valid value 0xFAFF (0xFB00..0xFFFF are J1939
// reserved / error / not-available). Callers place the result low-byte-first.

// 0.03125 deg C/bit, offset -273 deg C. Used by SPN 1172 (compressor intake
// temp), 1180 (turbine intake temp), 2629 (compressor outlet temp).
uint16_t j1939EncodeTemperatureRaw(float tempC);

// SPN 1127 Engine Turbocharger 1 Boost Pressure: 0.125 kPa/bit, offset 0.
// Input is gauge boost in hPa (compressor pressure rise).
uint16_t j1939EncodeBoostRaw(float boostHpa);

// SPN 1176 (compressor intake pressure) / 1209 (exhaust gas pressure):
// 1/128 kPa/bit, offset -250 kPa. Input pressure in hPa.
uint16_t j1939EncodeTurboPressureRaw(float pressureHpa);

#endif
