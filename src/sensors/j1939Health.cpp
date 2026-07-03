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

static bool seenWithin(uint32_t lastMs, uint32_t nowMs, uint32_t timeoutMs) {
    return lastMs != 0 && (uint32_t)(nowMs - lastMs) <= timeoutMs;
}

bool j1939DomainOnline(uint32_t lastGateA, uint32_t lastGateB,
                       uint32_t nowMs, uint32_t timeoutMs) {
    return seenWithin(lastGateA, nowMs, timeoutMs)
        || seenWithin(lastGateB, nowMs, timeoutMs);
}

J1939Status j1939SignalStatus(bool domainOnline, uint32_t lastSeenMs,
                              uint32_t nowMs, uint32_t timeoutMs,
                              J1939RawStatus raw) {
    if (!domainOnline) return J1939_STATUS_WAITING;
    if (!seenWithin(lastSeenMs, nowMs, timeoutMs)) return J1939_STATUS_ABSENT;
    switch (raw) {
        case J1939_NA:  return J1939_STATUS_NA;
        case J1939_ERR: return J1939_STATUS_ERR;
        default:        return J1939_STATUS_OK;
    }
}

const char *j1939StatusName(J1939Status s) {
    switch (s) {
        case J1939_STATUS_NA:      return "na";
        case J1939_STATUS_ERR:     return "err";
        case J1939_STATUS_ABSENT:  return "absent";
        case J1939_STATUS_WAITING: return "waiting";
        default:                   return "ok";
    }
}
