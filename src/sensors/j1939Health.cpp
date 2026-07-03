#include "j1939Health.h"

J1939RawStatus j1939RawStatus1Byte(uint8_t raw) {
    if (raw == 0xFF) return J1939_NA;
    if (raw == 0xFE) return J1939_ERR;
    return J1939_OK;
}

J1939RawStatus j1939RawStatus2Byte(uint16_t raw) {
    if (raw >= 0xFF00) return J1939_NA;
    if (raw >= 0xFE00) return J1939_ERR;  // 0xFE00..0xFEFF (0xFF00+ already returned)
    return J1939_OK;
}
