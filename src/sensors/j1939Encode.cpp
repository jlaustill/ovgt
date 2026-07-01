#include "j1939Encode.h"

// Clamp a computed raw value to the valid J1939 2-byte range. 0xFB00..0xFFFF are
// reserved for error / not-available, so the highest valid measurement is 0xFAFF.
static uint16_t clampRaw(float value) {
    if (value < 0.0f) return 0;
    if (value > 64255.0f) return 0xFAFF;  // 0xFAFF
    return (uint16_t)(value + 0.5f);
}

uint16_t j1939EncodeTemperatureRaw(float tempC) {
    // 0.03125 deg C/bit, offset -273  =>  (tempC + 273) / 0.03125 = (tempC + 273) * 32
    return clampRaw((tempC + 273.0f) * 32.0f);
}

uint16_t j1939EncodeBoostRaw(float boostHpa) {
    // 0.125 kPa/bit, offset 0. hPa -> kPa is * 0.1, then / 0.125  =>  hPa * 0.8
    return clampRaw(boostHpa * 0.8f);
}

uint16_t j1939EncodeTurboPressureRaw(float pressureHpa) {
    // 1/128 kPa/bit, offset -250 kPa.  ((hPa * 0.1) + 250) / (1/128) = (kPa + 250) * 128
    return clampRaw((pressureHpa * 0.1f + 250.0f) * 128.0f);
}
