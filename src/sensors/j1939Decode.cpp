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
