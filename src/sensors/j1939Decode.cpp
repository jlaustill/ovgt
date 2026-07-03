#include "j1939Decode.h"

uint32_t pgnFromCanId(uint32_t canId) {
    uint8_t pf = (uint8_t)((canId >> 16) & 0xFF);
    uint8_t ps = (uint8_t)((canId >> 8) & 0xFF);
    if (pf >= 240) {
        return ((uint32_t)pf << 8) | ps;
    }
    return (uint32_t)pf << 8;
}

uint8_t saFromCanId(uint32_t canId) {
    return (uint8_t)(canId & 0xFF);
}

uint8_t decodeTorqueConverterLockup(const uint8_t *buf) {
    return (uint8_t)((buf[0] >> 2) & 0x03);
}

uint8_t decodeAcceleratorPedalPercent(const uint8_t *buf) {
    float percent = buf[1] * 0.4f;
    if (percent > 100.0f) percent = 100.0f;
    return (uint8_t)(percent + 0.5f);
}

uint8_t decodeEngineLoadPercent(const uint8_t *buf) {
    return buf[2];  // 1 %/bit, range already 0-250
}

uint16_t decodeEngineRpm(const uint8_t *buf) {
    uint16_t raw = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
    return (uint16_t)(raw * 0.125f + 0.5f);
}

int8_t decodeDriverDemandTorquePct(const uint8_t *buf) {
    return (int8_t)((int)buf[1] - 125);
}

int8_t decodeActualTorquePct(const uint8_t *buf) {
    return (int8_t)((int)buf[2] - 125);
}

int16_t decodeIntakeAirTempC(const uint8_t *buf) { return (int16_t)((int)buf[2] - 40); }
uint16_t decodeBoostKpa(const uint8_t *buf)      { return (uint16_t)buf[1] * 2; }
uint16_t decodePreTurboKpa(const uint8_t *buf)   { return (uint16_t)(buf[0] * 0.5f + 0.5f); }

int16_t decodeCoolantTempC(const uint8_t *buf) { return (int16_t)((int)buf[0] - 40); }
int16_t decodeOilTempC(const uint8_t *buf) {
    uint16_t raw = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
    return (int16_t)(raw * 0.03125f - 273.0f + (raw ? 0.5f : 0.0f));
}
uint16_t decodeOilPressureKpa(const uint8_t *buf) { return (uint16_t)buf[3] * 4; }

float decodeSystemVoltage(const uint8_t *buf) {
    uint16_t raw = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
    return raw * 0.05f;
}

uint16_t decodeOutputShaftRpm(const uint8_t *buf) {
    uint16_t raw = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
    return (uint16_t)(raw * 0.125f + 0.5f);
}

uint16_t decodeInputShaftRpm(const uint8_t *buf) {
    uint16_t raw = (uint16_t)buf[5] | ((uint16_t)buf[6] << 8);
    return (uint16_t)(raw * 0.125f + 0.5f);
}

uint8_t decodeClutchSlipPct(const uint8_t *buf) {
    return (uint8_t)(buf[3] * 0.4f + 0.5f);
}

uint8_t decodeSelectedGear(const uint8_t *buf) { return buf[0]; }
uint8_t decodeCurrentGear(const uint8_t *buf)  { return buf[3]; }
uint16_t decodeGearRatioMilli(const uint8_t *buf) {
    return (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
}
