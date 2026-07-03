# J1939 Full-Bus Logging Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Capture the engine + transmission J1939 broadcast into OVGT's telemetry stream, with per-signal FF/FE/absent health detection, a broadcast-online gate, and unknown-PGN discovery, to test the exhaust-energy (H1) and brown-out-reboot (H2) hypotheses.

**Architecture:** Extend the existing `j1939Sniff` ISR (`src/sensors/j1939.cpp`) to decode target PGNs into `AppData`, filtered by CAN source address. Pure decoders + a pure health/online state machine live in `src/sensors/` and are unit-tested natively (Unity). The 10 Hz `"t"` telemetry line gains the decoded values; a new 1 Hz `"d"` line carries signal health + online flags; one `"u"` line per undecoded PGN carries the discovery inventory. Host (`tools/ovgt-telemetry`) routes the new message types into new Mongo collections; decoded `"t"` fields flow in automatically via the existing `{...sample}` spread.

**Tech Stack:** C++ / Arduino / Teensy 4.1 / FlexCAN_T4 / PlatformIO; C-Next (`json.cnx`); host TypeScript + tsx + vitest + MongoDB.

## Global Constraints

- Firmware only logs raw signals — NO mass-flow/energy math, NO control changes, NO new J1939 TX (spec §Out of scope).
- Decode a PGN only when its CAN source address matches: engine = `0x00`, transmission = `0x03`. OVGT transmits 65270/65262/65263 itself at SA `0x01` — never decode those.
- `engine_online` gates on EEC1 (61444) or EEC2 (61443); `trans_online` gates on ETC1 (61442) or ETC2 (61445). OCT synthesizes IC1/ET1/EFL-P1 on SA `0x00` but NOT EEC1/EEC2 — those are native-engine-only.
- Signal health is 5-state: `ok | na (0xFF) | err (0xFE) | absent | waiting`. `waiting` = domain offline (pre-unlock); `absent` = domain online but signal stale. `timeout = max(3 × broadcastRate, 1000 ms)`.
- J1939 multi-byte fields are little-endian. NA/err on raw bytes: 1-byte → `0xFF` na, `0xFE` err; 2-byte → `raw ≥ 0xFF00` na, `0xFE00–0xFEFF` err.
- `j1939Sniff` runs in ISR context: no `Serial`, no allocation, keep light; shared `AppData` fields written from the ISR are `volatile`. All JSON emission happens in the main loop (`ovgt.cpp`).
- Newly tested `src/` files must be added to `[env:native]` `build_src_filter` in `platformio.ini` (project CLAUDE.md).
- Generated C-Next files (`json.c`/`json.h`) are committed; regenerate with `python cnext_build.py` (project CLAUDE.md).
- Byte-offset source of truth for cross-check: `/home/linux/code/oct/src/data/cm848-j1939-receiver.cpp`.

---

## File Structure

- `src/sensors/j1939Decode.{h,cpp}` — MODIFY. Add `saFromCanId` + one decoder per new SPN. Pure, native-tested.
- `src/sensors/j1939Health.{h,cpp}` — CREATE. Pure raw-status classifiers + online-gate + signal-status state machine. Native-tested.
- `include/AppData.h` — MODIFY. New value fields, raw-status fields, per-PGN `lastSeen`, domain-online latch timestamps, unknown-PGN inventory table.
- `src/sensors/j1939.cpp` — MODIFY. Extend `j1939Sniff` (SA filter, decode, stamp, gate, tally); expose getters for the diagnostic builder.
- `src/domain/json.cnx` (+ regenerated `json.c`/`json.h`) — MODIFY. Bump buffer 640 → 1280.
- `src/domain/ovgt.cpp` — MODIFY. Add decoded fields to the `"t"` line; emit `"d"` and `"u"` lines at 1 Hz.
- `test/test_j1939_decode/test_j1939_decode.cpp` — MODIFY. Add decoder + `saFromCanId` cases.
- `test/test_j1939_health/test_j1939_health.cpp` — CREATE. Health/online state-machine cases.
- `platformio.ini` — MODIFY. Add `j1939Health.cpp` to `[env:native]` `build_src_filter`.
- `tools/ovgt-telemetry/src/{types,parse,store}.ts` (+ `.test.ts`) — MODIFY. Route/store `"d"` and `"u"`.

**Message → collection mapping (flat, no nested JSON — builder is flat):**
- `"t"` (10 Hz) → `telemetry` (existing; gains decoded value fields, `null` when not `ok`).
- `"d"` (1 Hz) → `j1939_diag`: `{t_ms, engine_online, trans_online, engine_up_ms, trans_up_ms, h_<signal>:"<status>"…}`.
- `"u"` (≤ few/s) → `j1939_unknown`: one doc per undecoded PGN `{t_ms, pgn, sa, cnt, hz, last}`.

---

### Task 1: Source-address extractor + raw-status classifiers (pure)

**Files:**
- Create: `src/sensors/j1939Health.h`, `src/sensors/j1939Health.cpp`
- Modify: `src/sensors/j1939Decode.h`, `src/sensors/j1939Decode.cpp`
- Modify: `platformio.ini` (`[env:native]` build_src_filter)
- Test: `test/test_j1939_health/test_j1939_health.cpp` (create)

**Interfaces:**
- Produces: `uint8_t saFromCanId(uint32_t canId)`; `enum J1939RawStatus { J1939_OK=0, J1939_NA=1, J1939_ERR=2 }`; `J1939RawStatus j1939RawStatus1Byte(uint8_t raw)`; `J1939RawStatus j1939RawStatus2Byte(uint16_t raw)`.

- [ ] **Step 1: Add `saFromCanId` declaration + definition**

In `src/sensors/j1939Decode.h`, after the `pgnFromCanId` declaration:
```cpp
// Extract the source address (low byte) from a 29-bit J1939 CAN identifier.
uint8_t saFromCanId(uint32_t canId);
```
In `src/sensors/j1939Decode.cpp`, after `pgnFromCanId`:
```cpp
uint8_t saFromCanId(uint32_t canId) {
    return (uint8_t)(canId & 0xFF);
}
```

- [ ] **Step 2: Create the pure health header**

Create `src/sensors/j1939Health.h`:
```cpp
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
```

- [ ] **Step 3: Write failing tests for the classifiers**

Create `test/test_j1939_health/test_j1939_health.cpp`:
```cpp
#include <unity.h>
#include "sensors/j1939Health.h"

void setUp(void) {}
void tearDown(void) {}

void test_raw1_ok(void)  { TEST_ASSERT_EQUAL_INT(J1939_OK,  j1939RawStatus1Byte(0x64)); }
void test_raw1_na(void)  { TEST_ASSERT_EQUAL_INT(J1939_NA,  j1939RawStatus1Byte(0xFF)); }
void test_raw1_err(void) { TEST_ASSERT_EQUAL_INT(J1939_ERR, j1939RawStatus1Byte(0xFE)); }

void test_raw2_ok(void)        { TEST_ASSERT_EQUAL_INT(J1939_OK,  j1939RawStatus2Byte(0x2000)); }
void test_raw2_na_boundary(void)  { TEST_ASSERT_EQUAL_INT(J1939_NA,  j1939RawStatus2Byte(0xFF00)); }
void test_raw2_na_full(void)      { TEST_ASSERT_EQUAL_INT(J1939_NA,  j1939RawStatus2Byte(0xFFFF)); }
void test_raw2_err_low(void)      { TEST_ASSERT_EQUAL_INT(J1939_ERR, j1939RawStatus2Byte(0xFE00)); }
void test_raw2_err_high(void)     { TEST_ASSERT_EQUAL_INT(J1939_ERR, j1939RawStatus2Byte(0xFEFF)); }

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_raw1_ok);
    RUN_TEST(test_raw1_na);
    RUN_TEST(test_raw1_err);
    RUN_TEST(test_raw2_ok);
    RUN_TEST(test_raw2_na_boundary);
    RUN_TEST(test_raw2_na_full);
    RUN_TEST(test_raw2_err_low);
    RUN_TEST(test_raw2_err_high);
    return UNITY_END();
}
```

- [ ] **Step 4: Add `j1939Health.cpp` to the native build filter**

In `platformio.ini`, `[env:native]` `build_src_filter`, append ` +<sensors/j1939Health.cpp>` after `+<sensors/j1939Decode.cpp>`.

- [ ] **Step 5: Run tests to verify they fail**

Run: `pio test -e native -f test_j1939_health`
Expected: FAIL — link error / `j1939RawStatus1Byte` undefined (no `.cpp` yet).

- [ ] **Step 6: Implement the classifiers**

Create `src/sensors/j1939Health.cpp` (only the two classifiers for now; rest added in Task 8):
```cpp
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
```

- [ ] **Step 7: Run tests to verify they pass**

Run: `pio test -e native -f test_j1939_health`
Expected: PASS (8 tests).

- [ ] **Step 8: Commit**
```bash
git add src/sensors/j1939Health.h src/sensors/j1939Health.cpp src/sensors/j1939Decode.h src/sensors/j1939Decode.cpp platformio.ini test/test_j1939_health/
git commit -m "feat(j1939): source-address extractor + raw FF/FE status classifiers"
```

---

### Task 2: EEC1 decoders — RPM, driver-demand & actual torque (pure)

**Files:**
- Modify: `src/sensors/j1939Decode.h`, `src/sensors/j1939Decode.cpp`
- Test: `test/test_j1939_decode/test_j1939_decode.cpp`

**Interfaces:**
- Produces: `uint16_t decodeEngineRpm(const uint8_t *buf)` (bytes 3-4 LE × 0.125 rpm/bit → rounded rpm); `int8_t decodeDriverDemandTorquePct(const uint8_t *buf)` (byte1 − 125); `int8_t decodeActualTorquePct(const uint8_t *buf)` (byte2 − 125).

- [ ] **Step 1: Write failing tests**

Add to `test/test_j1939_decode/test_j1939_decode.cpp` (before `main`):
```cpp
void test_engine_rpm(void) {
    // 1500 rpm / 0.125 = 12000 = 0x2EE0 -> LE bytes 3=0xE0, 4=0x2E
    uint8_t buf[8] = {0,0,0, 0xE0, 0x2E, 0,0,0};
    TEST_ASSERT_EQUAL_UINT16(1500, decodeEngineRpm(buf));
}
void test_driver_demand_torque(void) {
    uint8_t buf[8] = {0, 150, 0,0,0,0,0,0};  // 150 - 125 = 25
    TEST_ASSERT_EQUAL_INT8(25, decodeDriverDemandTorquePct(buf));
}
void test_actual_torque_negative(void) {
    uint8_t buf[8] = {0,0, 100, 0,0,0,0,0};  // 100 - 125 = -25 (motoring)
    TEST_ASSERT_EQUAL_INT8(-25, decodeActualTorquePct(buf));
}
```
Add matching `RUN_TEST(...)` lines inside `main`'s `UNITY_BEGIN()` block.

- [ ] **Step 2: Run to verify fail**

Run: `pio test -e native -f test_j1939_decode`
Expected: FAIL — `decodeEngineRpm` undefined.

- [ ] **Step 3: Declare + implement**

In `j1939Decode.h`:
```cpp
// EEC1 (PGN 61444) bytes 3-4 LE -> SPN 190 Engine Speed (0.125 rpm/bit).
uint16_t decodeEngineRpm(const uint8_t *buf);
// EEC1 byte 1 -> SPN 512 Driver's Demand Percent Torque (1 %/bit, offset -125).
int8_t decodeDriverDemandTorquePct(const uint8_t *buf);
// EEC1 byte 2 -> SPN 513 Actual Engine Percent Torque (1 %/bit, offset -125).
int8_t decodeActualTorquePct(const uint8_t *buf);
```
In `j1939Decode.cpp`:
```cpp
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
```

- [ ] **Step 4: Run to verify pass**

Run: `pio test -e native -f test_j1939_decode`
Expected: PASS (existing + 3 new).

- [ ] **Step 5: Commit**
```bash
git add src/sensors/j1939Decode.h src/sensors/j1939Decode.cpp test/test_j1939_decode/
git commit -m "feat(j1939): decode EEC1 RPM + driver-demand/actual torque"
```

---

### Task 3: IC1 + AMB decoders — intake-air temp, boost, pre-turbo pressure (pure)

**Files:** Modify `src/sensors/j1939Decode.{h,cpp}`; Test `test/test_j1939_decode/test_j1939_decode.cpp`

**Interfaces:**
- Produces: `int16_t decodeIntakeAirTempC(const uint8_t *buf)` (IC1 byte2 − 40); `uint16_t decodeBoostKpa(const uint8_t *buf)` (IC1 byte1 × 2); `uint16_t decodePreTurboKpa(const uint8_t *buf)` (AMB 65269 byte0 × 0.5, rounded).

- [ ] **Step 1: Write failing tests**
```cpp
void test_intake_air_temp(void) {
    uint8_t buf[8] = {0,0, 80, 0,0,0,0,0};  // 80 - 40 = 40 C
    TEST_ASSERT_EQUAL_INT16(40, decodeIntakeAirTempC(buf));
}
void test_boost_kpa(void) {
    uint8_t buf[8] = {0, 100, 0,0,0,0,0,0};  // 100 * 2 = 200 kPa
    TEST_ASSERT_EQUAL_UINT16(200, decodeBoostKpa(buf));
}
void test_preturbo_kpa(void) {
    uint8_t buf[8] = {200, 0,0,0,0,0,0,0};  // 200 * 0.5 = 100 kPa
    TEST_ASSERT_EQUAL_UINT16(100, decodePreTurboKpa(buf));
}
```
Add the three `RUN_TEST` lines.

- [ ] **Step 2: Run to verify fail** — `pio test -e native -f test_j1939_decode` → FAIL.

- [ ] **Step 3: Declare + implement**

`j1939Decode.h`:
```cpp
// IC1 (PGN 65270) byte 2 -> SPN 105 Intake Manifold 1 Temp (1 C/bit, offset -40).
int16_t decodeIntakeAirTempC(const uint8_t *buf);
// IC1 byte 1 -> SPN 102 Boost Pressure (2 kPa/bit).
uint16_t decodeBoostKpa(const uint8_t *buf);
// AMB (PGN 65269) byte 0 -> SPN 108 Barometric/pre-turbo pressure (0.5 kPa/bit).
uint16_t decodePreTurboKpa(const uint8_t *buf);
```
`j1939Decode.cpp`:
```cpp
int16_t decodeIntakeAirTempC(const uint8_t *buf) { return (int16_t)((int)buf[2] - 40); }
uint16_t decodeBoostKpa(const uint8_t *buf)      { return (uint16_t)buf[1] * 2; }
uint16_t decodePreTurboKpa(const uint8_t *buf)   { return (uint16_t)(buf[0] * 0.5f + 0.5f); }
```

- [ ] **Step 4: Run to verify pass** — PASS.
- [ ] **Step 5: Commit**
```bash
git commit -am "feat(j1939): decode IC1 intake-air temp/boost + AMB pre-turbo pressure"
```

---

### Task 4: ET1 + EFL/P1 decoders — coolant temp, oil temp, oil pressure (pure)

**Files:** Modify `src/sensors/j1939Decode.{h,cpp}`; Test same test file

**Interfaces:**
- Produces: `int16_t decodeCoolantTempC(const uint8_t *buf)` (ET1 byte0 − 40); `int16_t decodeOilTempC(const uint8_t *buf)` (ET1 bytes2-3 LE × 0.03125 − 273); `uint16_t decodeOilPressureKpa(const uint8_t *buf)` (EFL/P1 byte3 × 4).

- [ ] **Step 1: Write failing tests**
```cpp
void test_coolant_temp(void) {
    uint8_t buf[8] = {130, 0,0,0,0,0,0,0};  // 130 - 40 = 90 C
    TEST_ASSERT_EQUAL_INT16(90, decodeCoolantTempC(buf));
}
void test_oil_temp(void) {
    // 100 C -> (100+273)/0.03125 = 11936 = 0x2EA0 -> LE byte2=0xA0, byte3=0x2E
    uint8_t buf[8] = {0,0, 0xA0, 0x2E, 0,0,0,0};
    TEST_ASSERT_EQUAL_INT16(100, decodeOilTempC(buf));
}
void test_oil_pressure(void) {
    uint8_t buf[8] = {0,0,0, 75, 0,0,0,0};  // 75 * 4 = 300 kPa
    TEST_ASSERT_EQUAL_UINT16(300, decodeOilPressureKpa(buf));
}
```
Add `RUN_TEST` lines.

- [ ] **Step 2: Run to verify fail** → FAIL.

- [ ] **Step 3: Declare + implement**

`j1939Decode.h`:
```cpp
// ET1 (PGN 65262) byte 0 -> SPN 110 Coolant Temp (1 C/bit, offset -40).
int16_t decodeCoolantTempC(const uint8_t *buf);
// ET1 bytes 2-3 LE -> SPN 175 Engine Oil Temp 1 (0.03125 C/bit, offset -273).
int16_t decodeOilTempC(const uint8_t *buf);
// EFL/P1 (PGN 65263) byte 3 -> SPN 100 Engine Oil Pressure (4 kPa/bit).
uint16_t decodeOilPressureKpa(const uint8_t *buf);
```
`j1939Decode.cpp`:
```cpp
int16_t decodeCoolantTempC(const uint8_t *buf) { return (int16_t)((int)buf[0] - 40); }
int16_t decodeOilTempC(const uint8_t *buf) {
    uint16_t raw = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
    return (int16_t)(raw * 0.03125f - 273.0f + (raw ? 0.5f : 0.0f));
}
uint16_t decodeOilPressureKpa(const uint8_t *buf) { return (uint16_t)buf[3] * 4; }
```

- [ ] **Step 4: Run to verify pass** — PASS.
- [ ] **Step 5: Commit**
```bash
git commit -am "feat(j1939): decode ET1 coolant/oil temp + EFL-P1 oil pressure"
```

---

### Task 5: VEP1 decoder — system/battery voltage (pure)

**Files:** Modify `src/sensors/j1939Decode.{h,cpp}`; Test same file

**Interfaces:**
- Produces: `float decodeSystemVoltage(const uint8_t *buf)` (VEP1 bytes 4-5 LE × 0.05 V/bit).

- [ ] **Step 1: Write failing test**
```cpp
void test_system_voltage(void) {
    // 13.8 V / 0.05 = 276 = 0x0114 -> LE byte4=0x14, byte5=0x01
    uint8_t buf[8] = {0,0,0,0, 0x14, 0x01, 0,0};
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 13.8f, decodeSystemVoltage(buf));
}
```
Add `RUN_TEST(test_system_voltage);`.

- [ ] **Step 2: Run to verify fail** → FAIL.

- [ ] **Step 3: Declare + implement**

`j1939Decode.h`:
```cpp
// VEP1 (PGN 65271) bytes 4-5 LE -> SPN 168 Battery Potential (0.05 V/bit).
float decodeSystemVoltage(const uint8_t *buf);
```
`j1939Decode.cpp`:
```cpp
float decodeSystemVoltage(const uint8_t *buf) {
    uint16_t raw = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
    return raw * 0.05f;
}
```

- [ ] **Step 4: Run to verify pass** — PASS.
- [ ] **Step 5: Commit**
```bash
git commit -am "feat(j1939): decode VEP1 system/battery voltage (SPN 168)"
```

---

### Task 6: ETC1 extra decoders — output/input shaft speed, clutch slip (pure)

**Files:** Modify `src/sensors/j1939Decode.{h,cpp}`; Test same file

**Interfaces:**
- Produces: `uint16_t decodeOutputShaftRpm(const uint8_t *buf)` (ETC1 bytes 1-2 LE × 0.125); `uint16_t decodeInputShaftRpm(const uint8_t *buf)` (ETC1 bytes 5-6 LE × 0.125); `uint8_t decodeClutchSlipPct(const uint8_t *buf)` (ETC1 byte3 × 0.4).

> Byte offsets follow SAE ETC1 and are confirmed for SPN 191 (bytes 1-2) by `oct/src/data/cummins-bus.cpp`. Cross-check SPN 161/522 offsets against `oct/src/data/cm848-j1939-receiver.cpp`; if oct differs, use oct's and update the test.

- [ ] **Step 1: Write failing tests**
```cpp
void test_output_shaft_rpm(void) {
    // 2000 rpm / 0.125 = 16000 = 0x3E80 -> LE byte1=0x80, byte2=0x3E
    uint8_t buf[8] = {0, 0x80, 0x3E, 0,0,0,0,0};
    TEST_ASSERT_EQUAL_UINT16(2000, decodeOutputShaftRpm(buf));
}
void test_input_shaft_rpm(void) {
    // 2400 rpm / 0.125 = 19200 = 0x4B00 -> LE byte5=0x00, byte6=0x4B
    uint8_t buf[8] = {0,0,0,0,0, 0x00, 0x4B, 0};
    TEST_ASSERT_EQUAL_UINT16(2400, decodeInputShaftRpm(buf));
}
void test_clutch_slip(void) {
    uint8_t buf[8] = {0,0,0, 25, 0,0,0,0};  // 25 * 0.4 = 10 %
    TEST_ASSERT_EQUAL_UINT8(10, decodeClutchSlipPct(buf));
}
```
Add `RUN_TEST` lines.

- [ ] **Step 2: Run to verify fail** → FAIL.

- [ ] **Step 3: Declare + implement**

`j1939Decode.h`:
```cpp
// ETC1 (PGN 61442) bytes 1-2 LE -> SPN 191 Output Shaft Speed (0.125 rpm/bit).
uint16_t decodeOutputShaftRpm(const uint8_t *buf);
// ETC1 bytes 5-6 LE -> SPN 161 Input Shaft Speed (0.125 rpm/bit).
uint16_t decodeInputShaftRpm(const uint8_t *buf);
// ETC1 byte 3 -> SPN 522 Percent Clutch Slip (0.4 %/bit).
uint8_t decodeClutchSlipPct(const uint8_t *buf);
```
`j1939Decode.cpp`:
```cpp
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
```

- [ ] **Step 4: Run to verify pass** — PASS.
- [ ] **Step 5: Commit**
```bash
git commit -am "feat(j1939): decode ETC1 output/input shaft speed + clutch slip"
```

---

### Task 7: ETC2 decoders — selected gear, current gear, gear ratio (pure)

**Files:** Modify `src/sensors/j1939Decode.{h,cpp}`; Test same file

**Interfaces:**
- Produces: `uint8_t decodeSelectedGear(const uint8_t *buf)` (ETC2 byte0, raw); `uint8_t decodeCurrentGear(const uint8_t *buf)` (ETC2 byte3, raw); `uint16_t decodeGearRatioMilli(const uint8_t *buf)` (ETC2 bytes 1-2 LE, 0.001/bit → integer milli-ratio).

> ETC2 gear bytes use Allison-specific range values (see `oct/docs/superpowers/specs/2026-06-10-allison-etc2-prnd-translation-design.md`); for H2 shift-event detection we log the raw gear bytes (transitions are what matter) and the SAE gear ratio.

- [ ] **Step 1: Write failing tests**
```cpp
void test_selected_gear_raw(void) {
    uint8_t buf[8] = {0x7E, 0,0,0,0,0,0,0};  // 0x7E = 1st (Allison)
    TEST_ASSERT_EQUAL_UINT8(0x7E, decodeSelectedGear(buf));
}
void test_current_gear_raw(void) {
    uint8_t buf[8] = {0,0,0, 0x7D, 0,0,0,0};  // byte3 = 0x7D = Neutral
    TEST_ASSERT_EQUAL_UINT8(0x7D, decodeCurrentGear(buf));
}
void test_gear_ratio_milli(void) {
    // 1.000 ratio -> 1000 = 0x03E8 -> LE byte1=0xE8, byte2=0x03
    uint8_t buf[8] = {0, 0xE8, 0x03, 0,0,0,0,0};
    TEST_ASSERT_EQUAL_UINT16(1000, decodeGearRatioMilli(buf));
}
```
Add `RUN_TEST` lines.

- [ ] **Step 2: Run to verify fail** → FAIL.

- [ ] **Step 3: Declare + implement**

`j1939Decode.h`:
```cpp
// ETC2 (PGN 61445) byte 0 -> SPN 524 Selected Gear (raw; Allison range-coded).
uint8_t decodeSelectedGear(const uint8_t *buf);
// ETC2 byte 3 -> SPN 523 Current Gear (raw; Allison range-coded).
uint8_t decodeCurrentGear(const uint8_t *buf);
// ETC2 bytes 1-2 LE -> SPN 526 Actual Gear Ratio (0.001/bit -> integer milli).
uint16_t decodeGearRatioMilli(const uint8_t *buf);
```
`j1939Decode.cpp`:
```cpp
uint8_t decodeSelectedGear(const uint8_t *buf) { return buf[0]; }
uint8_t decodeCurrentGear(const uint8_t *buf)  { return buf[3]; }
uint16_t decodeGearRatioMilli(const uint8_t *buf) {
    return (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
}
```

- [ ] **Step 4: Run to verify pass** — PASS.
- [ ] **Step 5: Commit**
```bash
git commit -am "feat(j1939): decode ETC2 selected/current gear + gear ratio"
```

---

### Task 8: Online-gate + signal-status state machine (pure)

**Files:** Modify `src/sensors/j1939Health.{h,cpp}`; Test `test/test_j1939_health/test_j1939_health.cpp`

**Interfaces:**
- Consumes: `J1939RawStatus`, `J1939Status` (Task 1).
- Produces: `bool j1939DomainOnline(...)`, `J1939Status j1939SignalStatus(...)`, `const char *j1939StatusName(J1939Status)` (signatures in `j1939Health.h` from Task 1 Step 2).

- [ ] **Step 1: Write failing tests**

Add to `test/test_j1939_health/test_j1939_health.cpp`:
```cpp
void test_domain_offline_never_seen(void) {
    TEST_ASSERT_FALSE(j1939DomainOnline(0, 0, 5000, 1000));
}
void test_domain_online_one_gate(void) {
    TEST_ASSERT_TRUE(j1939DomainOnline(4500, 0, 5000, 1000));  // gate A 500ms ago
}
void test_domain_offline_stale(void) {
    TEST_ASSERT_FALSE(j1939DomainOnline(3000, 0, 5000, 1000)); // 2000ms ago > 1000
}
void test_signal_waiting_when_domain_offline(void) {
    // domain offline -> WAITING regardless of raw
    TEST_ASSERT_EQUAL_INT(J1939_STATUS_WAITING,
        j1939SignalStatus(false, 4900, 5000, 1000, J1939_OK));
}
void test_signal_absent_when_online_but_never_seen(void) {
    TEST_ASSERT_EQUAL_INT(J1939_STATUS_ABSENT,
        j1939SignalStatus(true, 0, 5000, 1000, J1939_OK));
}
void test_signal_absent_when_online_but_stale(void) {
    TEST_ASSERT_EQUAL_INT(J1939_STATUS_ABSENT,
        j1939SignalStatus(true, 3000, 5000, 1000, J1939_OK));
}
void test_signal_ok_when_fresh(void) {
    TEST_ASSERT_EQUAL_INT(J1939_STATUS_OK,
        j1939SignalStatus(true, 4800, 5000, 1000, J1939_OK));
}
void test_signal_na_maps_through(void) {
    TEST_ASSERT_EQUAL_INT(J1939_STATUS_NA,
        j1939SignalStatus(true, 4800, 5000, 1000, J1939_NA));
}
void test_signal_err_maps_through(void) {
    TEST_ASSERT_EQUAL_INT(J1939_STATUS_ERR,
        j1939SignalStatus(true, 4800, 5000, 1000, J1939_ERR));
}
void test_status_names(void) {
    TEST_ASSERT_EQUAL_STRING("ok",      j1939StatusName(J1939_STATUS_OK));
    TEST_ASSERT_EQUAL_STRING("na",      j1939StatusName(J1939_STATUS_NA));
    TEST_ASSERT_EQUAL_STRING("err",     j1939StatusName(J1939_STATUS_ERR));
    TEST_ASSERT_EQUAL_STRING("absent",  j1939StatusName(J1939_STATUS_ABSENT));
    TEST_ASSERT_EQUAL_STRING("waiting", j1939StatusName(J1939_STATUS_WAITING));
}
```
Add all the `RUN_TEST(...)` lines to `main`.

- [ ] **Step 2: Run to verify fail** — `pio test -e native -f test_j1939_health` → FAIL (undefined refs).

- [ ] **Step 3: Implement the state machine**

Append to `src/sensors/j1939Health.cpp`:
```cpp
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
```

- [ ] **Step 4: Run to verify pass** — PASS (all health tests).
- [ ] **Step 5: Commit**
```bash
git commit -am "feat(j1939): online-gate + 5-state signal-status state machine"
```

---

### Task 9: AppData fields + ISR decode/gate/inventory wiring

**Files:** Modify `include/AppData.h`, `src/sensors/j1939.cpp`

**Interfaces:**
- Consumes: all decoders (Tasks 2-7), `saFromCanId`, `j1939RawStatus*` (Task 1).
- Produces (for Tasks 11-13): populated `AppData` fields + `J1939::getDiagJson`-feeding getters. Specifically new `AppData` members listed below, and `J1939::unknownCount()` / `J1939::unknownAt(i, out)`.

This task is integration (ISR + hardware) — verified by native compile of pure parts (already green) plus a Teensy build. No new unit test; the on-truck smoke drive (Task 16) is its behavioral test.

- [ ] **Step 1: Add AppData fields**

In `include/AppData.h`, inside `struct AppData`, after `lastEtc1RxMs`:
```cpp
    // --- J1939 broadcast logging (decoded in j1939Sniff; see logging spec) ---
    volatile uint16_t engineRpm;              // SPN 190
    volatile int8_t   driverDemandTorquePct;  // SPN 512
    volatile int8_t   actualTorquePct;        // SPN 513
    volatile int16_t  intakeAirTempC;         // SPN 105
    volatile uint16_t j1939BoostKpa;          // SPN 102 (cross-check)
    volatile uint16_t preTurboKpa;            // SPN 108
    volatile float    systemVoltage;          // SPN 168
    volatile int16_t  coolantTempC;           // SPN 110
    volatile int16_t  engineOilTempC;         // SPN 175
    volatile uint16_t oilPressureKpaJ1939;    // SPN 100
    volatile uint16_t outputShaftRpm;         // SPN 191
    volatile uint16_t inputShaftRpm;          // SPN 161
    volatile uint8_t  clutchSlipPct;          // SPN 522
    volatile uint8_t  selectedGear;           // SPN 524 (raw gear byte value)
    volatile uint8_t  currentGear;            // SPN 523 (raw gear byte value)
    volatile uint16_t gearRatioMilli;         // SPN 526
    // Raw FF/FE J1939 validity status per signal (J1939RawStatus as uint8_t; ISR-set).
    // Convention: <signal>Raw holds the *status*; the plain field holds the *value*.
    volatile uint8_t  engineRpmRaw, torqueRaw, intakeAirRaw, boostRaw, preTurboRaw,
                      systemVoltageRaw, coolantRaw, oilTempRaw, oilPressRaw,
                      outputShaftRaw, inputShaftRaw, clutchSlipRaw,
                      selectedGearRaw, currentGearRaw, gearRatioRaw;
    // Per-PGN last-receipt millis() for freshness/online gating
    volatile uint32_t lastEec1RxMs, lastIc1RxMs, lastAmbRxMs, lastVep1RxMs,
                      lastEt1RxMs, lastEflRxMs, lastEtc2RxMs;
    // Domain-online latch timestamps (set once on first native gating frame; 0 = not yet)
    volatile uint32_t engineOnlineAtMs, transOnlineAtMs;
```

- [ ] **Step 2: Add PGN constants + unknown-inventory table to `j1939.cpp`**

In `src/sensors/j1939.cpp`, after the existing PGN constants (line ~16), add:
```cpp
static const uint32_t PGN_EEC1  = 61444;
static const uint32_t PGN_IC1   = 65270;  // engine SA 0x00 (distinct from OVGT's own TX)
static const uint32_t PGN_AMB65269 = 65269;
static const uint32_t PGN_VEP1  = 65271;
static const uint32_t PGN_ET1   = 65262;
static const uint32_t PGN_EFLP1 = 65263;
static const uint32_t PGN_ETC2  = 61445;

static const uint8_t SA_ENGINE = 0x00;
static const uint8_t SA_TRANS  = 0x03;

// Unknown-PGN discovery inventory (ISR-updated, main-loop-read).
struct UnknownPgn {
    uint32_t pgn;
    uint8_t  sa;
    uint32_t count;
    uint32_t firstMs;
    uint32_t lastMs;
    uint8_t  last[8];
};
static const uint8_t UNKNOWN_MAX = 32;
static volatile UnknownPgn unknownTable[UNKNOWN_MAX];
static volatile uint8_t unknownUsed = 0;
static volatile uint32_t unknownDropped = 0;

static void tallyUnknown(uint32_t pgn, uint8_t sa, const uint8_t *buf, uint32_t now) {
    for (uint8_t i = 0; i < unknownUsed; i++) {
        if (unknownTable[i].pgn == pgn && unknownTable[i].sa == sa) {
            unknownTable[i].count++;
            unknownTable[i].lastMs = now;
            for (uint8_t b = 0; b < 8; b++) unknownTable[i].last[b] = buf[b];
            return;
        }
    }
    if (unknownUsed >= UNKNOWN_MAX) { unknownDropped++; return; }
    uint8_t i = unknownUsed++;
    unknownTable[i].pgn = pgn; unknownTable[i].sa = sa;
    unknownTable[i].count = 1; unknownTable[i].firstMs = now; unknownTable[i].lastMs = now;
    for (uint8_t b = 0; b < 8; b++) unknownTable[i].last[b] = buf[b];
}
```

- [ ] **Step 3: Rewrite `j1939Sniff` with SA filtering, decode, gating, and tally**

Replace the body of `j1939Sniff` (`src/sensors/j1939.cpp:37-47`):
```cpp
static void j1939Sniff(const CAN_message_t &msg) {
    uint32_t pgn = pgnFromCanId(msg.id);
    uint8_t  sa  = saFromCanId(msg.id);
    uint32_t now = millis();

    if (sa == SA_ENGINE) {
        switch (pgn) {
            case PGN_EEC1:
                appData.engineRpm = decodeEngineRpm(msg.buf);
                appData.engineRpmRaw = j1939RawStatus2Byte((uint16_t)msg.buf[3] | ((uint16_t)msg.buf[4] << 8));
                appData.driverDemandTorquePct = decodeDriverDemandTorquePct(msg.buf);
                appData.actualTorquePct = decodeActualTorquePct(msg.buf);
                appData.torqueRaw = j1939RawStatus1Byte(msg.buf[2]);
                appData.lastEec1RxMs = now;
                if (appData.engineOnlineAtMs == 0) appData.engineOnlineAtMs = now;
                return;
            case PGN_EEC2:
                appData.acceleratorPedalPercent = decodeAcceleratorPedalPercent(msg.buf);
                appData.engineLoadPercent = decodeEngineLoadPercent(msg.buf);
                appData.lastEec2RxMs = now;
                if (appData.engineOnlineAtMs == 0) appData.engineOnlineAtMs = now;
                return;
            case PGN_IC1:
                appData.intakeAirTempC = decodeIntakeAirTempC(msg.buf);
                appData.intakeAirRaw = j1939RawStatus1Byte(msg.buf[2]);
                appData.j1939BoostKpa = decodeBoostKpa(msg.buf);
                appData.boostRaw = j1939RawStatus1Byte(msg.buf[1]);
                appData.lastIc1RxMs = now;
                return;
            case PGN_AMB65269:
                appData.preTurboKpa = decodePreTurboKpa(msg.buf);
                appData.preTurboRaw = j1939RawStatus1Byte(msg.buf[0]);
                appData.lastAmbRxMs = now;
                return;
            case PGN_VEP1:
                appData.systemVoltage = decodeSystemVoltage(msg.buf);
                appData.systemVoltageRaw = j1939RawStatus2Byte((uint16_t)msg.buf[4] | ((uint16_t)msg.buf[5] << 8));
                appData.lastVep1RxMs = now;
                return;
            case PGN_ET1:
                appData.coolantTempC = decodeCoolantTempC(msg.buf);
                appData.coolantRaw = j1939RawStatus1Byte(msg.buf[0]);
                appData.engineOilTempC = decodeOilTempC(msg.buf);
                appData.oilTempRaw = j1939RawStatus2Byte((uint16_t)msg.buf[2] | ((uint16_t)msg.buf[3] << 8));
                appData.lastEt1RxMs = now;
                return;
            case PGN_EFLP1:
                appData.oilPressureKpaJ1939 = decodeOilPressureKpa(msg.buf);
                appData.oilPressRaw = j1939RawStatus1Byte(msg.buf[3]);
                appData.lastEflRxMs = now;
                return;
        }
    } else if (sa == SA_TRANS) {
        switch (pgn) {
            case PGN_ETC1:
                appData.torqueConverterLockupStatus = decodeTorqueConverterLockup(msg.buf);
                appData.outputShaftRpm = decodeOutputShaftRpm(msg.buf);
                appData.outputShaftRaw = j1939RawStatus2Byte((uint16_t)msg.buf[1] | ((uint16_t)msg.buf[2] << 8));
                appData.inputShaftRpm = decodeInputShaftRpm(msg.buf);
                appData.inputShaftRaw = j1939RawStatus2Byte((uint16_t)msg.buf[5] | ((uint16_t)msg.buf[6] << 8));
                appData.clutchSlipPct = decodeClutchSlipPct(msg.buf);
                appData.clutchSlipRaw = j1939RawStatus1Byte(msg.buf[3]);
                appData.lastEtc1RxMs = now;
                if (appData.transOnlineAtMs == 0) appData.transOnlineAtMs = now;
                return;
            case PGN_ETC2:
                appData.selectedGear = decodeSelectedGear(msg.buf);
                appData.selectedGearRaw = j1939RawStatus1Byte(msg.buf[0]);
                appData.currentGear = decodeCurrentGear(msg.buf);
                appData.currentGearRaw = j1939RawStatus1Byte(msg.buf[3]);
                appData.gearRatioMilli = decodeGearRatioMilli(msg.buf);
                appData.gearRatioRaw = j1939RawStatus2Byte((uint16_t)msg.buf[1] | ((uint16_t)msg.buf[2] << 8));
                appData.lastEtc2RxMs = now;
                if (appData.transOnlineAtMs == 0) appData.transOnlineAtMs = now;
                return;
        }
    }
    // Undecoded frame from any SA (skip our own SA 0x01 transmissions).
    if (sa != SA) tallyUnknown(pgn, sa, msg.buf, now);
}
```
Add includes at the top of `j1939.cpp`: `#include "j1939Health.h"`.

- [ ] **Step 4: Add inventory getters for the emit tasks**

In `src/sensors/j1939.h`, add to the `J1939` class/namespace declaration:
```cpp
    static uint8_t unknownCount();
    static bool unknownAt(uint8_t i, uint32_t &pgn, uint8_t &sa, uint32_t &count,
                          uint32_t &firstMs, uint32_t &lastMs, uint8_t out8[8]);
    static uint32_t unknownDroppedCount();
```
In `src/sensors/j1939.cpp` (main-context reads; snapshot volatile fields):
```cpp
uint8_t J1939::unknownCount() { return unknownUsed; }
uint32_t J1939::unknownDroppedCount() { return unknownDropped; }
bool J1939::unknownAt(uint8_t i, uint32_t &pgn, uint8_t &sa, uint32_t &count,
                      uint32_t &firstMs, uint32_t &lastMs, uint8_t out8[8]) {
    if (i >= unknownUsed) return false;
    noInterrupts();
    pgn = unknownTable[i].pgn; sa = unknownTable[i].sa; count = unknownTable[i].count;
    firstMs = unknownTable[i].firstMs; lastMs = unknownTable[i].lastMs;
    for (uint8_t b = 0; b < 8; b++) out8[b] = unknownTable[i].last[b];
    interrupts();
    return true;
}
```

- [ ] **Step 5: Build for Teensy to verify it compiles**

Run: `pio run -e teensy41`
Expected: SUCCESS (compiles + links). Fix any type/naming mismatches against Step 1's field names.

- [ ] **Step 6: Commit**
```bash
git add include/AppData.h src/sensors/j1939.cpp src/sensors/j1939.h
git commit -m "feat(j1939): SA-filtered decode of engine+trans PGNs, online latch, unknown-PGN inventory"
```

---

### Task 10: Bump the C-Next JSON buffer (640 → 1280)

**Files:** Modify `src/domain/json.cnx`; regenerate `src/domain/json.c`, `src/domain/json.h`

The `"t"` line grows to ~39 fields (~700 B) and would overflow the 639-byte guard. Bump the buffer.

- [ ] **Step 1: Locate the buffer + guard in `json.cnx`**

Inspect: `grep -n "639\|640\|buf\[" src/domain/json.cnx` — find the backing array size (`640`) and the `i < 639` room check in `putByte`.

- [ ] **Step 2: Change size + guard**

Edit `src/domain/json.cnx`: change the backing array length `640` → `1280` and the guard `i < 639` → `i < 1279`. (Match the exact identifiers found in Step 1.)

- [ ] **Step 3: Regenerate the C output**

Run: `python cnext_build.py`
Expected: rewrites `src/domain/json.c` and `src/domain/json.h`; the array/guard constants now read 1280/1279.

- [ ] **Step 4: Verify native + Teensy build**

Run: `pio test -e native -f test_j1939_health` then `pio run -e teensy41`
Expected: both PASS/SUCCESS (regenerated `json.c` still compiles in both envs).

- [ ] **Step 5: Commit**
```bash
git add src/domain/json.cnx src/domain/json.c src/domain/json.h
git commit -m "feat(json): grow NDJSON buffer 640->1280 for expanded telemetry line"
```

---

### Task 11: Add decoded J1939 fields to the 10 Hz `"t"` line

**Files:** Modify `src/domain/ovgt.cpp`

Emit each decoded value, or JSON `null` when its status is not `ok`, so analysis can trust non-null values. Uses the same per-signal status resolution as the `"d"` line (Task 12) but inline.

- [ ] **Step 1: Add a status helper near the top of `ovgt.cpp`**

After the includes, add:
```cpp
#include "sensors/j1939Health.h"

// Per-signal 5-state status from AppData freshness + raw status.
static J1939Status jStatus(bool domainOnline, uint32_t lastSeen, uint32_t timeoutMs, uint8_t rawStatus) {
    return j1939SignalStatus(domainOnline, lastSeen, millis(), timeoutMs, (J1939RawStatus)rawStatus);
}
```

- [ ] **Step 2: Emit the new fields in `handleDebug`**

In `src/domain/ovgt.cpp:handleDebug`, before `Json_end();` (line 85), add (uses `Json_addNull` — see Step 3):
```cpp
    bool engineOnline = j1939DomainOnline(appData.lastEec1RxMs, appData.lastEec2RxMs, millis(), 1000);
    bool transOnline  = j1939DomainOnline(appData.lastEtc1RxMs, appData.lastEtc2RxMs, millis(), 1000);

    if (jStatus(engineOnline, appData.lastEec1RxMs, 1000, appData.engineRpmRaw) == J1939_STATUS_OK)
        Json_addUint("engine_rpm", appData.engineRpm);
    else Json_addNull("engine_rpm");
    // ...repeat the pattern for each signal below (value key, timeout, lastSeen, raw-status):
    //   torque_pct       (int, actualTorquePct, lastEec1RxMs, 1000, torqueRaw)
    //   torque_demand_pct(int, driverDemandTorquePct, lastEec1RxMs, 1000, torqueRaw)
    //   accel_pct        (uint, acceleratorPedalPercent, lastEec2RxMs, 1000, OK-always: EEC2 has no single raw byte tracked -> use engineOnline+freshness only)
    //   load_pct         (uint, engineLoadPercent, lastEec2RxMs, 1000)
    //   intake_air_c     (int, intakeAirTempC, lastIc1RxMs, 1500, intakeAirRaw)
    //   j1939_boost_kpa  (uint, j1939BoostKpa, lastIc1RxMs, 1500, boostRaw)
    //   preturbo_kpa     (uint, preTurboKpa, lastAmbRxMs, 3000, preTurboRaw)
    //   system_v         (float2, systemVoltage, lastVep1RxMs, 3000, systemVoltageRaw)
    //   coolant_c        (int, coolantTempC, lastEt1RxMs, 3000, coolantRaw)
    //   oil_c            (int, engineOilTempC, lastEt1RxMs, 3000, oilTempRaw)
    //   oil_kpa          (uint, oilPressureKpaJ1939, lastEflRxMs, 1500, oilPressRaw)
    //   tcc              (uint, torqueConverterLockupStatus, lastEtc1RxMs, 1000; transOnline)
    //   trans_out_rpm    (uint, outputShaftRpm, lastEtc1RxMs, 1000, outputShaftRaw; transOnline)
    //   trans_in_rpm     (uint, inputShaftRpm, lastEtc1RxMs, 1000, inputShaftRaw; transOnline)
    //   clutch_slip_pct  (uint, clutchSlipPct, lastEtc1RxMs, 1000, clutchSlipRaw; transOnline)
    //   gear_sel         (uint, selectedGear, lastEtc2RxMs, 1000, selectedGearRaw; transOnline)
    //   gear_cur         (uint, currentGear, lastEtc2RxMs, 1000, currentGearRaw; transOnline)
    //   gear_ratio       (uint, gearRatioMilli, lastEtc2RxMs, 1000, gearRatioRaw; transOnline)
```
Write out each line explicitly following the `engine_rpm` template — one `if (jStatus(...)==OK) Json_add<T>(...); else Json_addNull(...);` per signal. Engine signals use `engineOnline`; trans signals use `transOnline`. For `accel_pct`/`load_pct`/`tcc` (no tracked raw byte) gate on domain-online + freshness only: `Json_add...` when the domain is online and the frame is fresh, else `Json_addNull`.

- [ ] **Step 3: Add `Json_addNull` to the JSON builder**

`Json_addNull` does not exist yet. In `src/domain/json.cnx`, add a public method mirroring `addUint` that writes `key:null`:
```
public void addNull(const string<24> k) {
    key(k);
    putRaw("null");
}
```
Regenerate: `python cnext_build.py` (updates `json.c`/`json.h` with `Json_addNull`).

- [ ] **Step 4: Build for Teensy**

Run: `pio run -e teensy41`
Expected: SUCCESS. (Optionally spot-check the emitted line length stays < 1279.)

- [ ] **Step 5: Commit**
```bash
git add src/domain/ovgt.cpp src/domain/json.cnx src/domain/json.c src/domain/json.h
git commit -m "feat(telemetry): add decoded J1939 signals to 10Hz line (null when not ok)"
```

---

### Task 12: Emit the 1 Hz `"d"` diagnostic line (health + online flags)

**Files:** Modify `src/domain/ovgt.cpp`

**Interfaces:**
- Consumes: `jStatus` (Task 11), `j1939StatusName`, `J1939::` inventory getters, AppData fields.

- [ ] **Step 1: Add a diagnostic emitter function**

In `src/domain/ovgt.cpp`, add (declare `void handleJ1939Diag();` in `ovgt.h`):
```cpp
void ovgt::handleJ1939Diag() {
    if (count % 100 != 0) return;  // 1 Hz (loop 100 Hz)
    uint32_t now = millis();
    bool eng = j1939DomainOnline(appData.lastEec1RxMs, appData.lastEec2RxMs, now, 1000);
    bool trn = j1939DomainOnline(appData.lastEtc1RxMs, appData.lastEtc2RxMs, now, 1000);

    Json_begin();
    Json_addStr("type", "d");
    Json_addUint("t_ms", now);
    Json_addBool("engine_online", eng);
    Json_addBool("trans_online", trn);
    Json_addUint("engine_up_ms", appData.engineOnlineAtMs);
    Json_addUint("trans_up_ms", appData.transOnlineAtMs);
    Json_addStr("h_engine_rpm",  j1939StatusName(jStatus(eng, appData.lastEec1RxMs, 1000, appData.engineRpmRaw)));
    Json_addStr("h_torque",      j1939StatusName(jStatus(eng, appData.lastEec1RxMs, 1000, appData.torqueRaw)));
    Json_addStr("h_intake_air",  j1939StatusName(jStatus(eng, appData.lastIc1RxMs, 1500, appData.intakeAirRaw)));
    Json_addStr("h_boost",       j1939StatusName(jStatus(eng, appData.lastIc1RxMs, 1500, appData.boostRaw)));
    Json_addStr("h_preturbo",    j1939StatusName(jStatus(eng, appData.lastAmbRxMs, 3000, appData.preTurboRaw)));
    Json_addStr("h_system_v",    j1939StatusName(jStatus(eng, appData.lastVep1RxMs, 3000, appData.systemVoltageRaw)));
    Json_addStr("h_coolant",     j1939StatusName(jStatus(eng, appData.lastEt1RxMs, 3000, appData.coolantRaw)));
    Json_addStr("h_oil_c",       j1939StatusName(jStatus(eng, appData.lastEt1RxMs, 3000, appData.oilTempRaw)));
    Json_addStr("h_oil_kpa",     j1939StatusName(jStatus(eng, appData.lastEflRxMs, 1500, appData.oilPressRaw)));
    Json_addStr("h_trans_out",   j1939StatusName(jStatus(trn, appData.lastEtc1RxMs, 1000, appData.outputShaftRaw)));
    Json_addStr("h_trans_in",    j1939StatusName(jStatus(trn, appData.lastEtc1RxMs, 1000, appData.inputShaftRaw)));
    Json_addStr("h_clutch_slip", j1939StatusName(jStatus(trn, appData.lastEtc1RxMs, 1000, appData.clutchSlipRaw)));
    Json_addStr("h_gear_sel",    j1939StatusName(jStatus(trn, appData.lastEtc2RxMs, 1000, appData.selectedGearRaw)));
    Json_addStr("h_gear_cur",    j1939StatusName(jStatus(trn, appData.lastEtc2RxMs, 1000, appData.currentGearRaw)));
    Json_addStr("h_gear_ratio",  j1939StatusName(jStatus(trn, appData.lastEtc2RxMs, 1000, appData.gearRatioRaw)));
    Json_addUint("unk_n", J1939::unknownCount());
    Json_addUint("unk_dropped", J1939::unknownDroppedCount());
    Json_end();
    for (uint32_t i = 0; i < Json_len(); i++) Serial.write(Json_at(i));
    Serial.write('\n');
}
```

- [ ] **Step 2: Call it from the main loop**

In `src/domain/ovgt.cpp:loop()`, after `handleDebug()` is invoked (find where `handleDebug`/`count` are pumped), add a call to `handleJ1939Diag();`. If `handleDebug` is called from a scheduler by `count`, place `handleJ1939Diag()` immediately alongside it.

- [ ] **Step 3: Build for Teensy** — `pio run -e teensy41` → SUCCESS.

- [ ] **Step 4: Commit**
```bash
git commit -am "feat(telemetry): 1Hz J1939 diagnostic line (health + online flags)"
```

---

### Task 13: Emit `"u"` unknown-PGN discovery lines

**Files:** Modify `src/domain/ovgt.cpp`

Emit one `"u"` line per known unknown-PGN entry, round-robin a few per diagnostic tick to bound serial load.

- [ ] **Step 1: Add the emitter**

In `src/domain/ovgt.cpp` (declare `void handleJ1939Unknown();` in `ovgt.h`):
```cpp
void ovgt::handleJ1939Unknown() {
    if (count % 100 != 0) return;  // 1 Hz, aligned with diag
    static uint8_t cursor = 0;
    uint8_t n = J1939::unknownCount();
    if (n == 0) return;
    uint8_t emit = n < 4 ? n : 4;  // up to 4 per second -> full 32-entry table in 8s
    for (uint8_t k = 0; k < emit; k++) {
        uint8_t idx = (cursor + k) % n;
        uint32_t pgn, count32, firstMs, lastMs; uint8_t sa, b[8];
        if (!J1939::unknownAt(idx, pgn, sa, count32, firstMs, lastMs, b)) continue;
        char hex[17];
        static const char HEXD[] = "0123456789ABCDEF";
        for (uint8_t j = 0; j < 8; j++) { hex[j*2] = HEXD[b[j] >> 4]; hex[j*2+1] = HEXD[b[j] & 0xF]; }
        hex[16] = '\0';
        uint32_t spanMs = lastMs - firstMs;
        uint32_t hz = spanMs > 0 ? (count32 * 1000) / spanMs : 0;
        Json_begin();
        Json_addStr("type", "u");
        Json_addUint("t_ms", (uint32_t)millis());
        Json_addUint("pgn", pgn);
        Json_addUint("sa", sa);
        Json_addUint("cnt", count32);
        Json_addUint("hz", hz);
        Json_addStr("last", hex);
        Json_end();
        for (uint32_t i = 0; i < Json_len(); i++) Serial.write(Json_at(i));
        Serial.write('\n');
    }
    cursor = (cursor + emit) % n;
}
```

- [ ] **Step 2: Call from loop** — add `handleJ1939Unknown();` next to `handleJ1939Diag();`.

- [ ] **Step 3: Build for Teensy** — `pio run -e teensy41` → SUCCESS.

- [ ] **Step 4: Commit**
```bash
git commit -am "feat(telemetry): emit unknown-PGN discovery lines (type u)"
```

---

### Task 14: Host — parse `"d"` and `"u"` message types

**Files:** Modify `tools/ovgt-telemetry/src/types.ts`, `parse.ts`, `parse.test.ts`

**Interfaces:**
- Consumes: existing `parse.ts` discriminator on `type` (`parse.ts:19`).
- Produces: `{kind:"j1939diag", doc}` and `{kind:"j1939unknown", doc}` parse results.

- [ ] **Step 1: Add types**

In `tools/ovgt-telemetry/src/types.ts`, add interfaces mirroring existing telemetry/settle types:
```ts
export interface J1939DiagDoc { type: "d"; t_ms: number; engine_online: boolean; trans_online: boolean; [k: string]: unknown; }
export interface J1939UnknownDoc { type: "u"; t_ms: number; pgn: number; sa: number; cnt: number; hz: number; last: string; }
```
Add `"j1939diag"` and `"j1939unknown"` to the parse-result union (follow the existing `"telemetry"`/`"settle"` union shape).

- [ ] **Step 2: Write failing tests**

In `tools/ovgt-telemetry/src/parse.test.ts`:
```ts
it("routes a type d diagnostic line", () => {
  const r = parseLine(JSON.stringify({ type: "d", t_ms: 1, engine_online: true, trans_online: false, h_engine_rpm: "ok" }));
  expect(r).toEqual({ kind: "j1939diag", doc: expect.objectContaining({ engine_online: true, h_engine_rpm: "ok" }) });
});
it("routes a type u unknown-PGN line", () => {
  const r = parseLine(JSON.stringify({ type: "u", t_ms: 1, pgn: 65247, sa: 0, cnt: 5, hz: 20, last: "FF00" }));
  expect(r).toEqual({ kind: "j1939unknown", doc: expect.objectContaining({ pgn: 65247 }) });
});
```
(Match the exact `parseLine` name/signature used in the existing tests.)

- [ ] **Step 3: Run to verify fail** — `cd tools/ovgt-telemetry && npm test -- parse` → FAIL.

- [ ] **Step 4: Implement routing**

In `parse.ts`, extend the `type` switch (near `:19`): `case "d": return { kind: "j1939diag", doc };` and `case "u": return { kind: "j1939unknown", doc };`.

- [ ] **Step 5: Run to verify pass** — `npm test -- parse` → PASS. Then `npm run typecheck` → clean.

- [ ] **Step 6: Commit**
```bash
git add tools/ovgt-telemetry/src/types.ts tools/ovgt-telemetry/src/parse.ts tools/ovgt-telemetry/src/parse.test.ts
git commit -m "feat(host): parse J1939 diagnostic (d) + unknown-PGN (u) lines"
```

---

### Task 15: Host — store `"d"` and `"u"` into new collections

**Files:** Modify `tools/ovgt-telemetry/src/store.ts`, `store.test.ts`

**Interfaces:**
- Consumes: `{kind:"j1939diag"|"j1939unknown", doc}` (Task 14); existing `Store` with `sessionId`, `connect()` index setup (`store.ts:27-37`), `insertTelemetry` (`store.ts:41-43`).
- Produces: `insertJ1939Diag(doc)`, `insertJ1939Unknown(doc)`, and fire-and-forget `recordJ1939Diag`/`recordJ1939Unknown` (mirror `recordTelemetry` at `store.ts:55`).

- [ ] **Step 1: Write failing tests** (needs local MongoDB per project CLAUDE.md)

In `tools/ovgt-telemetry/src/store.test.ts`, mirroring the existing settle-event store test:
```ts
it("inserts a j1939 diag doc with sessionId + ts", async () => {
  const store = new Store(uri);
  await store.connect("test", "host");
  await store.insertJ1939Diag({ type: "d", t_ms: 1, engine_online: true, trans_online: false });
  const doc = await store.db.collection("j1939_diag").findOne({});
  expect(doc).toMatchObject({ engine_online: true });
  expect(doc.sessionId).toBeDefined();
  expect(doc.ts).toBeInstanceOf(Date);
  await store.close();
});
it("inserts a j1939 unknown doc", async () => {
  const store = new Store(uri);
  await store.connect("test", "host");
  await store.insertJ1939Unknown({ type: "u", t_ms: 1, pgn: 65247, sa: 0, cnt: 5, hz: 20, last: "FF" });
  const doc = await store.db.collection("j1939_unknown").findOne({});
  expect(doc).toMatchObject({ pgn: 65247 });
  await store.close();
});
```
(Match the existing test's `Store` construction, `uri`, and accessor conventions.)

- [ ] **Step 2: Run to verify fail** — `npm test -- store` → FAIL (methods undefined).

- [ ] **Step 3: Implement**

In `store.ts`, mirror `insertTelemetry` (`:41-43`):
```ts
async insertJ1939Diag(doc: J1939DiagDoc) {
  await this.db.collection("j1939_diag").insertOne({ ...doc, ts: new Date(), sessionId: this.sessionId });
}
async insertJ1939Unknown(doc: J1939UnknownDoc) {
  await this.db.collection("j1939_unknown").insertOne({ ...doc, ts: new Date(), sessionId: this.sessionId });
}
```
Add fire-and-forget `recordJ1939Diag`/`recordJ1939Unknown` mirroring `recordTelemetry` (`:55`). In `connect()` index setup (`:31`), add:
```ts
await this.db.collection("j1939_diag").createIndex({ sessionId: 1, ts: 1 });
await this.db.collection("j1939_unknown").createIndex({ sessionId: 1, ts: 1 });
```
Wire the new `kind`s into the app dispatch (`app.tsx`/`index.tsx`) next to the settle-event handling: `j1939diag` → `recordJ1939Diag`, `j1939unknown` → `recordJ1939Unknown`.

- [ ] **Step 4: Run to verify pass** — `npm test -- store` → PASS. `npm run typecheck` → clean.

- [ ] **Step 5: Commit**
```bash
git add tools/ovgt-telemetry/src/store.ts tools/ovgt-telemetry/src/store.test.ts tools/ovgt-telemetry/src/app.tsx tools/ovgt-telemetry/src/index.tsx
git commit -m "feat(host): store J1939 diag + unknown-PGN docs in dedicated collections"
```

---

### Task 16: On-truck smoke drive — verify capture end-to-end

**Files:** none (validation). Produces the first dataset for H1/H2 analysis.

- [ ] **Step 1: Flash + full build**

Run: `pio run -e teensy41 -t upload`
Expected: upload succeeds.

- [ ] **Step 2: Confirm native + host suites are green**

Run: `pio test -e native` and `cd tools/ovgt-telemetry && npm test && npm run typecheck`
Expected: all PASS.

- [ ] **Step 3: Capture a short session** with the host tool running (engine started, a couple of gear shifts, a light boost pull), then inspect Mongo:
  - `telemetry`: `engine_rpm` non-null and tracks throttle; `system_v` ~13–14 V; `gear_cur` changes on shifts.
  - `j1939_diag`: `engine_online` flips to `true` shortly after start; `engine_up_ms` records the delay; health mostly `ok`, `waiting` only before unlock.
  - `j1939_unknown`: lists extra broadcast PGNs (expect EEC3 65247, etc.).

- [ ] **Step 4: If engine signals stay `waiting`/`absent` all session**, that's a real finding (OCT never unlocked at OVGT's tap) — record it; it does not block the plan. Confirm the `unknown` inventory still populated (bus is alive).

- [ ] **Step 5: Commit any offset corrections** discovered by cross-checking captured `j1939_unknown.last` payloads / `oct` receiver against the decoder tests, then re-run `pio test -e native -f test_j1939_decode`.

---

## Notes for the analysis phase (not this plan)

Once ≥1 clean drive is captured, run the H1 energy-collapse test and H2 reboot/voltage/shift correlation per the spec's Validation Plan (`docs/superpowers/specs/2026-07-03-j1939-full-bus-logging-design.md`), building on the query patterns in `docs/analysis/2026-07-02-driving-log-review.md`.
