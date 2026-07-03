#ifndef j1939Health_h
#define j1939Health_h

#include <stdint.h>
#include <stdbool.h>

// Per-SPN validity derived from raw J1939 bytes (SAE J1939-71 SLOT convention).
enum J1939RawStatus { J1939_OK = 0, J1939_NA = 1, J1939_ERR = 2 };

// Full 5-state per-signal status after domain-online and freshness gating.
enum J1939Status {
    J1939_STATUS_OK = 0,
    J1939_STATUS_NA = 1,
    J1939_STATUS_ERR = 2,
    J1939_STATUS_ABSENT = 3,
    J1939_STATUS_WAITING = 4
};

// Raw-byte classification: 0xFF -> NA, 0xFE -> ERR, else OK.
J1939RawStatus j1939RawStatus1Byte(uint8_t raw);
// 2-byte little-endian: raw >= 0xFF00 -> NA, 0xFE00..0xFEFF -> ERR, else OK.
J1939RawStatus j1939RawStatus2Byte(uint16_t raw);

// A domain (engine/trans) is online if either gating PGN was seen within timeout.
// lastGateA/lastGateB are millis() of last receipt (0 = never).
bool j1939DomainOnline(uint32_t lastGateA, uint32_t lastGateB,
                       uint32_t nowMs, uint32_t timeoutMs);

// Resolve a signal's 5-state status. WAITING when domain offline; ABSENT when
// online but stale/never-seen; otherwise the raw status maps through.
J1939Status j1939SignalStatus(bool domainOnline, uint32_t lastSeenMs,
                              uint32_t nowMs, uint32_t timeoutMs,
                              J1939RawStatus raw);

// Lowercase status name for JSON emission: "ok"/"na"/"err"/"absent"/"waiting".
const char *j1939StatusName(J1939Status s);

#endif
