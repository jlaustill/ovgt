# Exhaust Brake Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a VGT-based exhaust brake that closes the turbine vanes under PI control to hold a target exhaust backpressure when the torque converter is locked and the engine is making no power.

**Architecture:** Pure, Arduino-free logic (J1939 decode + the brake state machine / PI loop) lives in separate files unit-tested natively with Unity. Thin Arduino "glue" (`ExhaustBrakeController`, `j1939Sniff`) reads `appData`/`millis()` and calls the pure functions. In `ovgt::loop()`, exactly one controller owns the actuator per 10 ms cycle: the brake when active, otherwise `BoostController`.

**Tech Stack:** C++ (Arduino/Teensy 4.1), PlatformIO 6.1.19, FlexCAN_T4, Unity test framework (PlatformIO `native` platform 1.2.1).

---

## File Structure

**New pure-logic files (no Arduino; compiled into both `teensy41` and `native`):**
- `src/sensors/j1939Decode.h` / `.cpp` — PGN extraction + SPN decode helpers.
- `src/control/exhaustBrakeLogic.h` / `.cpp` — `exhaustBrakeStep()` state machine + PI loop.

**New Arduino glue (compiled into `teensy41` only):**
- `src/control/exhaustBrakeController.h` / `.cpp` — reads `appData`, calls `exhaustBrakeStep()`.

**Modified:**
- `platformio.ini` — add `[env:native]`.
- `include/AppData.h` — add receive + telemetry fields.
- `src/sensors/j1939.cpp` — implement `j1939Sniff()` using the decode helpers.
- `src/domain/ovgt.cpp` — init, arbitration, debug line.

**New tests:**
- `test/test_smoke/test_smoke.cpp`
- `test/test_j1939_decode/test_j1939_decode.cpp`
- `test/test_exhaust_brake_logic/test_exhaust_brake_logic.cpp`

---

## Task 1: Native test environment + smoke test

**Files:**
- Modify: `platformio.ini`
- Test: `test/test_smoke/test_smoke.cpp`

- [ ] **Step 1: Add the native env to `platformio.ini`**

Append this block to `platformio.ini` (leave the existing `[env:teensy41]` unchanged):

```ini
[env:native]
platform = native
test_framework = unity
build_src_filter = -<*> +<control/exhaustBrakeLogic.cpp> +<sensors/j1939Decode.cpp>
build_flags = -I src -I include
```

- [ ] **Step 2: Write the smoke test**

Create `test/test_smoke/test_smoke.cpp`:

```cpp
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

void test_native_toolchain_works(void) {
    TEST_ASSERT_EQUAL_INT(2, 1 + 1);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_native_toolchain_works);
    return UNITY_END();
}
```

- [ ] **Step 3: Run the smoke test**

Run: `pio test -e native -f test_smoke`
Expected: PASS — `test_native_toolchain_works:PASS` and `1 Tests 0 Failures 0 Ignored`.

- [ ] **Step 4: Commit**

```bash
git add platformio.ini test/test_smoke/test_smoke.cpp
git commit -m "Add native unit test environment with smoke test

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 2: J1939 decode pure functions

**Files:**
- Create: `src/sensors/j1939Decode.h`, `src/sensors/j1939Decode.cpp`
- Test: `test/test_j1939_decode/test_j1939_decode.cpp`

- [ ] **Step 1: Write the header**

Create `src/sensors/j1939Decode.h`:

```cpp
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
```

- [ ] **Step 2: Write the failing tests**

Create `test/test_j1939_decode/test_j1939_decode.cpp`:

```cpp
#include <unity.h>
#include "sensors/j1939Decode.h"

void setUp(void) {}
void tearDown(void) {}

void test_pgn_eec2_pdu2(void) {
    // priority 6, PF 0xF0, PS 0x03, SA 0x00  -> PGN 61443
    uint32_t id = ((uint32_t)6 << 26) | ((uint32_t)0xF0 << 16) | ((uint32_t)0x03 << 8) | 0x00;
    TEST_ASSERT_EQUAL_UINT32(61443u, pgnFromCanId(id));
}

void test_pgn_etc1_pdu2(void) {
    // PF 0xF0, PS 0x02 -> PGN 61442
    uint32_t id = ((uint32_t)6 << 26) | ((uint32_t)0xF0 << 16) | ((uint32_t)0x02 << 8) | 0x21;
    TEST_ASSERT_EQUAL_UINT32(61442u, pgnFromCanId(id));
}

void test_pgn_pdu1_excludes_ps(void) {
    // PF 0xEF (239 < 240) -> PS ignored, PGN = 0xEF00
    uint32_t id = ((uint32_t)6 << 26) | ((uint32_t)0xEF << 16) | ((uint32_t)0x05 << 8) | 0x21;
    TEST_ASSERT_EQUAL_UINT32((uint32_t)0xEF00, pgnFromCanId(id));
}

void test_lockup_engaged(void) {
    uint8_t buf[8] = {0x04, 0, 0, 0, 0, 0, 0, 0};  // bits 3-4 = 0b01
    TEST_ASSERT_EQUAL_UINT8(1, decodeTorqueConverterLockup(buf));
}

void test_lockup_not_engaged(void) {
    uint8_t buf[8] = {0x00, 0, 0, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_UINT8(0, decodeTorqueConverterLockup(buf));
}

void test_lockup_error_state(void) {
    uint8_t buf[8] = {0x08, 0, 0, 0, 0, 0, 0, 0};  // bits 3-4 = 0b10
    TEST_ASSERT_EQUAL_UINT8(2, decodeTorqueConverterLockup(buf));
}

void test_accelerator_zero(void) {
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_UINT8(0, decodeAcceleratorPedalPercent(buf));
}

void test_accelerator_full(void) {
    uint8_t buf[8] = {0, 250, 0, 0, 0, 0, 0, 0};  // 250 * 0.4 = 100
    TEST_ASSERT_EQUAL_UINT8(100, decodeAcceleratorPedalPercent(buf));
}

void test_accelerator_half(void) {
    uint8_t buf[8] = {0, 125, 0, 0, 0, 0, 0, 0};  // 125 * 0.4 = 50
    TEST_ASSERT_EQUAL_UINT8(50, decodeAcceleratorPedalPercent(buf));
}

void test_engine_load(void) {
    uint8_t buf[8] = {0, 0, 100, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_UINT8(100, decodeEngineLoadPercent(buf));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_pgn_eec2_pdu2);
    RUN_TEST(test_pgn_etc1_pdu2);
    RUN_TEST(test_pgn_pdu1_excludes_ps);
    RUN_TEST(test_lockup_engaged);
    RUN_TEST(test_lockup_not_engaged);
    RUN_TEST(test_lockup_error_state);
    RUN_TEST(test_accelerator_zero);
    RUN_TEST(test_accelerator_full);
    RUN_TEST(test_accelerator_half);
    RUN_TEST(test_engine_load);
    return UNITY_END();
}
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `pio test -e native -f test_j1939_decode`
Expected: FAIL — link/compile error, `undefined reference to pgnFromCanId` (and the other functions), because `j1939Decode.cpp` does not exist yet.

- [ ] **Step 4: Write the implementation**

Create `src/sensors/j1939Decode.cpp`:

```cpp
#include "j1939Decode.h"

uint32_t pgnFromCanId(uint32_t canId) {
    uint8_t pf = (uint8_t)((canId >> 16) & 0xFF);
    uint8_t ps = (uint8_t)((canId >> 8) & 0xFF);
    if (pf >= 240) {
        return ((uint32_t)pf << 8) | ps;
    }
    return (uint32_t)pf << 8;
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
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `pio test -e native -f test_j1939_decode`
Expected: PASS — `10 Tests 0 Failures 0 Ignored`.

- [ ] **Step 6: Commit**

```bash
git add src/sensors/j1939Decode.h src/sensors/j1939Decode.cpp test/test_j1939_decode/test_j1939_decode.cpp
git commit -m "Add J1939 PGN/SPN decode helpers with native tests

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 3: Exhaust brake logic (state machine + PI loop)

**Files:**
- Create: `src/control/exhaustBrakeLogic.h`, `src/control/exhaustBrakeLogic.cpp`
- Test: `test/test_exhaust_brake_logic/test_exhaust_brake_logic.cpp`

- [ ] **Step 1: Write the header**

Create `src/control/exhaustBrakeLogic.h`:

```cpp
#ifndef exhaustBrakeLogic_h
#define exhaustBrakeLogic_h

#include <stdint.h>

struct BrakeInputs {
    uint8_t  torqueConverterLockupStatus;  // 0-3, 1 = engaged
    uint8_t  acceleratorPedalPercent;      // 0-100
    uint8_t  engineLoadPercent;            // 0-250
    float    tipGaugePsi;                  // measured backpressure (turbine inlet, gauge)
    uint32_t nowMs;
    uint32_t lastEec2RxMs;
    uint32_t lastEtc1RxMs;
    bool     manualMode;
};

struct BrakeConfig {
    float    targetPsi;
    float    ceilingPsi;
    uint8_t  minVanePercent;
    uint8_t  maxVanePercent;
    uint32_t staleTimeoutMs;
    float    kp;
    float    ki;
};

struct BrakeState {
    float integralTerm;  // accumulated integral CONTRIBUTION, in vane-% units
};

struct BrakeOutput {
    bool    active;       // true when the brake owns the actuator this cycle
    uint8_t vanePercent;  // demanded vane position (valid when active)
    bool    fault;        // true when the backpressure ceiling was exceeded
};

// Evaluate one brake control cycle. Priority order: manual override, stale-CAN
// failsafe, instant throttle/load release, engage condition, ceiling cutoff,
// then the PI loop. Mutates state.integralTerm.
BrakeOutput exhaustBrakeStep(const BrakeInputs &in, const BrakeConfig &cfg,
                             BrakeState &state, float dtSeconds);

#endif
```

- [ ] **Step 2: Write the failing tests**

Create `test/test_exhaust_brake_logic/test_exhaust_brake_logic.cpp`:

```cpp
#include <unity.h>
#include "control/exhaustBrakeLogic.h"

void setUp(void) {}
void tearDown(void) {}

static BrakeConfig makeConfig(void) {
    BrakeConfig cfg;
    cfg.targetPsi = 60.0f;
    cfg.ceilingPsi = 65.0f;
    cfg.minVanePercent = 18;
    cfg.maxVanePercent = 68;
    cfg.staleTimeoutMs = 250;
    cfg.kp = 0.5f;
    cfg.ki = 0.3f;
    return cfg;
}

// Baseline: brake fully engaged, fresh CAN, backpressure at zero.
static BrakeInputs makeEngagedInputs(void) {
    BrakeInputs in;
    in.torqueConverterLockupStatus = 1;
    in.acceleratorPedalPercent = 0;
    in.engineLoadPercent = 0;
    in.tipGaugePsi = 0.0f;
    in.nowMs = 1000;
    in.lastEec2RxMs = 1000;
    in.lastEtc1RxMs = 1000;
    in.manualMode = false;
    return in;
}

void test_manual_mode_disengages(void) {
    BrakeInputs in = makeEngagedInputs();
    in.manualMode = true;
    BrakeConfig cfg = makeConfig();
    BrakeState st = {5.0f};
    BrakeOutput out = exhaustBrakeStep(in, cfg, st, 0.01f);
    TEST_ASSERT_FALSE(out.active);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, st.integralTerm);  // integral reset
}

void test_stale_eec2_disengages(void) {
    BrakeInputs in = makeEngagedInputs();
    in.lastEec2RxMs = 700;  // 1000 - 700 = 300 > 250
    BrakeConfig cfg = makeConfig();
    BrakeState st = {0.0f};
    BrakeOutput out = exhaustBrakeStep(in, cfg, st, 0.01f);
    TEST_ASSERT_FALSE(out.active);
}

void test_stale_etc1_disengages(void) {
    BrakeInputs in = makeEngagedInputs();
    in.lastEtc1RxMs = 700;
    BrakeConfig cfg = makeConfig();
    BrakeState st = {0.0f};
    BrakeOutput out = exhaustBrakeStep(in, cfg, st, 0.01f);
    TEST_ASSERT_FALSE(out.active);
}

void test_throttle_releases(void) {
    BrakeInputs in = makeEngagedInputs();
    in.acceleratorPedalPercent = 10;
    BrakeConfig cfg = makeConfig();
    BrakeState st = {5.0f};
    BrakeOutput out = exhaustBrakeStep(in, cfg, st, 0.01f);
    TEST_ASSERT_FALSE(out.active);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, st.integralTerm);
}

void test_load_releases(void) {
    BrakeInputs in = makeEngagedInputs();
    in.engineLoadPercent = 5;
    BrakeConfig cfg = makeConfig();
    BrakeState st = {0.0f};
    BrakeOutput out = exhaustBrakeStep(in, cfg, st, 0.01f);
    TEST_ASSERT_FALSE(out.active);
}

void test_not_locked_inactive(void) {
    BrakeInputs in = makeEngagedInputs();
    in.torqueConverterLockupStatus = 0;
    BrakeConfig cfg = makeConfig();
    BrakeState st = {0.0f};
    BrakeOutput out = exhaustBrakeStep(in, cfg, st, 0.01f);
    TEST_ASSERT_FALSE(out.active);
}

void test_lockup_error_state_inactive(void) {
    BrakeInputs in = makeEngagedInputs();
    in.torqueConverterLockupStatus = 2;  // error state, not engaged
    BrakeConfig cfg = makeConfig();
    BrakeState st = {0.0f};
    BrakeOutput out = exhaustBrakeStep(in, cfg, st, 0.01f);
    TEST_ASSERT_FALSE(out.active);
}

void test_engaged_drives_vanes(void) {
    BrakeInputs in = makeEngagedInputs();  // tip = 0, error = 60
    BrakeConfig cfg = makeConfig();
    BrakeState st = {0.0f};
    BrakeOutput out = exhaustBrakeStep(in, cfg, st, 0.01f);
    TEST_ASSERT_TRUE(out.active);
    TEST_ASSERT_FALSE(out.fault);
    // vane = kp*error + integral = 0.5*60 + 0.3*60*0.01 = 30.18 -> 30
    TEST_ASSERT_EQUAL_UINT8(30, out.vanePercent);
}

void test_ceiling_opens_vanes(void) {
    BrakeInputs in = makeEngagedInputs();
    in.tipGaugePsi = 70.0f;  // above 65 ceiling
    BrakeConfig cfg = makeConfig();
    BrakeState st = {10.0f};
    BrakeOutput out = exhaustBrakeStep(in, cfg, st, 0.01f);
    TEST_ASSERT_TRUE(out.active);
    TEST_ASSERT_TRUE(out.fault);
    TEST_ASSERT_EQUAL_UINT8(18, out.vanePercent);  // forced open to minVane
}

void test_integral_antiwindup_clamps(void) {
    BrakeInputs in = makeEngagedInputs();  // persistent large error
    BrakeConfig cfg = makeConfig();
    BrakeState st = {0.0f};
    BrakeOutput out;
    for (int i = 0; i < 1000; i++) {
        out = exhaustBrakeStep(in, cfg, st, 0.01f);
    }
    TEST_ASSERT_TRUE(out.active);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 68.0f, st.integralTerm);  // clamped at maxVane
    TEST_ASSERT_EQUAL_UINT8(68, out.vanePercent);              // output clamped too
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_manual_mode_disengages);
    RUN_TEST(test_stale_eec2_disengages);
    RUN_TEST(test_stale_etc1_disengages);
    RUN_TEST(test_throttle_releases);
    RUN_TEST(test_load_releases);
    RUN_TEST(test_not_locked_inactive);
    RUN_TEST(test_lockup_error_state_inactive);
    RUN_TEST(test_engaged_drives_vanes);
    RUN_TEST(test_ceiling_opens_vanes);
    RUN_TEST(test_integral_antiwindup_clamps);
    return UNITY_END();
}
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `pio test -e native -f test_exhaust_brake_logic`
Expected: FAIL — `undefined reference to exhaustBrakeStep`, because `exhaustBrakeLogic.cpp` does not exist yet.

- [ ] **Step 4: Write the implementation**

Create `src/control/exhaustBrakeLogic.cpp`:

```cpp
#include "exhaustBrakeLogic.h"

static uint8_t clampVane(float vane, uint8_t lo, uint8_t hi) {
    if (vane < (float)lo) return lo;
    if (vane > (float)hi) return hi;
    return (uint8_t)(vane + 0.5f);
}

BrakeOutput exhaustBrakeStep(const BrakeInputs &in, const BrakeConfig &cfg,
                             BrakeState &state, float dtSeconds) {
    BrakeOutput out;
    out.active = false;
    out.vanePercent = cfg.minVanePercent;
    out.fault = false;

    // 1. Manual override yields to the existing serial/boost behavior.
    if (in.manualMode) {
        state.integralTerm = 0.0f;
        return out;
    }

    // 2. Stale-CAN failsafe: never act on stale lockup/throttle/load.
    bool eec2Stale = (in.nowMs - in.lastEec2RxMs) > cfg.staleTimeoutMs;
    bool etc1Stale = (in.nowMs - in.lastEtc1RxMs) > cfg.staleTimeoutMs;
    if (eec2Stale || etc1Stale) {
        state.integralTerm = 0.0f;
        return out;
    }

    // 3. Instant release: the driver always wins.
    if (in.acceleratorPedalPercent > 0 || in.engineLoadPercent > 0) {
        state.integralTerm = 0.0f;
        return out;
    }

    // 4. Engage condition: torque converter locked (status 0b01) and no power.
    bool locked = (in.torqueConverterLockupStatus == 0x01);
    if (!locked) {
        state.integralTerm = 0.0f;
        return out;
    }

    out.active = true;

    // 5. Ceiling cutoff (hardware protection against valve float): force vanes
    //    open, flag a fault, and hold the integral.
    if (in.tipGaugePsi > cfg.ceilingPsi) {
        out.fault = true;
        out.vanePercent = cfg.minVanePercent;
        return out;
    }

    // 6. PI loop. No derivative term: TIP is noisy and D would jitter the vanes.
    float error = cfg.targetPsi - in.tipGaugePsi;
    state.integralTerm += cfg.ki * error * dtSeconds;
    if (state.integralTerm < 0.0f) {
        state.integralTerm = 0.0f;
    }
    if (state.integralTerm > (float)cfg.maxVanePercent) {
        state.integralTerm = (float)cfg.maxVanePercent;  // anti-windup
    }
    float vane = cfg.kp * error + state.integralTerm;
    out.vanePercent = clampVane(vane, cfg.minVanePercent, cfg.maxVanePercent);
    return out;
}
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `pio test -e native -f test_exhaust_brake_logic`
Expected: PASS — `10 Tests 0 Failures 0 Ignored`.

- [ ] **Step 6: Commit**

```bash
git add src/control/exhaustBrakeLogic.h src/control/exhaustBrakeLogic.cpp test/test_exhaust_brake_logic/test_exhaust_brake_logic.cpp
git commit -m "Add exhaust brake control logic with native tests

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 4: Add AppData fields

**Files:**
- Modify: `include/AppData.h`

- [ ] **Step 1: Add the new fields**

In `include/AppData.h`, add these fields to the `AppData` struct, immediately after the existing `bool pgFault;` line:

```cpp
    // J1939-received engine/transmission state (decoded in j1939Sniff)
    uint8_t torqueConverterLockupStatus;  // SPN 573, 2-bit (0-3), 1 = engaged
    uint8_t acceleratorPedalPercent;      // SPN 91, 0-100
    uint8_t engineLoadPercent;            // SPN 92, 0-250
    volatile uint32_t lastEec2RxMs;       // millis() of last EEC2 (PGN 61443) receive
    volatile uint32_t lastEtc1RxMs;       // millis() of last ETC1 (PGN 61442) receive
    bool exhaustBrakeActive;              // true while the exhaust brake owns the actuator
```

- [ ] **Step 2: Verify it compiles for the target**

Run: `pio run -e teensy41`
Expected: SUCCESS — `[SUCCESS]`. (No behavior change yet; this only widens the struct.)

- [ ] **Step 3: Commit**

```bash
git add include/AppData.h
git commit -m "Add AppData fields for exhaust brake J1939 inputs and state

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 5: Wire j1939Sniff to decode the trigger PGNs

**Files:**
- Modify: `src/sensors/j1939.cpp:17-22` (the `j1939Sniff` stub)

- [ ] **Step 1: Add the decode include**

In `src/sensors/j1939.cpp`, add this include after the existing `#include "AppData.h"` line:

```cpp
#include "j1939Decode.h"
```

- [ ] **Step 2: Add the receive PGN constants**

In `src/sensors/j1939.cpp`, after the existing `PGN_FLUID_PRESS` definition, add:

```cpp
static const uint32_t PGN_EEC2 = 61443; // EEC2 — SPN 91 throttle, SPN 92 load
static const uint32_t PGN_ETC1 = 61442; // ETC1 — SPN 573 torque converter lockup
```

- [ ] **Step 3: Implement the `j1939Sniff` body**

Replace the entire `j1939Sniff` function body (currently the TODO stub and `(void)msg;`) with:

```cpp
static void j1939Sniff(const CAN_message_t &msg) {
    uint32_t pgn = pgnFromCanId(msg.id);
    if (pgn == PGN_EEC2) {
        appData.acceleratorPedalPercent = decodeAcceleratorPedalPercent(msg.buf);
        appData.engineLoadPercent = decodeEngineLoadPercent(msg.buf);
        appData.lastEec2RxMs = millis();
    } else if (pgn == PGN_ETC1) {
        appData.torqueConverterLockupStatus = decodeTorqueConverterLockup(msg.buf);
        appData.lastEtc1RxMs = millis();
    }
}
```

- [ ] **Step 4: Verify it compiles for the target**

Run: `pio run -e teensy41`
Expected: SUCCESS — `[SUCCESS]`.

- [ ] **Step 5: Commit**

```bash
git add src/sensors/j1939.cpp
git commit -m "Decode EEC2 and ETC1 from J1939 into AppData

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 6: ExhaustBrakeController glue

**Files:**
- Create: `src/control/exhaustBrakeController.h`, `src/control/exhaustBrakeController.cpp`

- [ ] **Step 1: Write the header**

Create `src/control/exhaustBrakeController.h`:

```cpp
#ifndef exhaustBrakeController_h
#define exhaustBrakeController_h

class ExhaustBrakeController {
    public:
        static void Initialize();
        // Returns true if the brake is active and has written
        // appData.actuatorDemandedPosition this cycle. manualMode is passed in
        // because it is private to the ovgt orchestrator.
        static bool update(bool manualMode);
};

#endif
```

- [ ] **Step 2: Write the implementation**

Create `src/control/exhaustBrakeController.cpp`:

```cpp
#include <Arduino.h>
#include "exhaustBrakeController.h"
#include "exhaustBrakeLogic.h"
#include "AppData.h"

static const float HPA_TO_PSI = 0.0145038f;

// --- Tunable configuration -------------------------------------------------
// WARNING: a target of >= ~55 psi REQUIRES 60 psi heavy-duty exhaust valve
// springs. With stock springs the exhaust valves float at/near 60 psi, losing
// braking and risking valvetrain damage. ceilingPsi is the valve-float
// hardware-protection limit for the 2004 5.9L 24-valve Cummins (~65 psi).
// kp/ki are starting estimates — tune on the truck.
static BrakeConfig brakeConfig = {
    60.0f,  // targetPsi
    65.0f,  // ceilingPsi (valve-float limit)
    18,     // minVanePercent (vanes open / boost baseline)
    68,     // maxVanePercent (actuator physical limit)
    250,    // staleTimeoutMs
    0.5f,   // kp (%vane per psi)
    0.3f    // ki (%vane per psi/s)
};

static BrakeState brakeState = {0.0f};
static uint32_t lastUpdateMs = 0;

void ExhaustBrakeController::Initialize() {
    brakeState.integralTerm = 0.0f;
    lastUpdateMs = millis();
    Serial.println("ExhaustBrakeController initialized");
}

bool ExhaustBrakeController::update(bool manualMode) {
    uint32_t now = millis();
    float dt = (now - lastUpdateMs) / 1000.0f;
    lastUpdateMs = now;
    if (dt <= 0.0f) dt = 0.01f;

    BrakeInputs in;
    in.torqueConverterLockupStatus = appData.torqueConverterLockupStatus;
    in.acceleratorPedalPercent = appData.acceleratorPedalPercent;
    in.engineLoadPercent = appData.engineLoadPercent;
    in.tipGaugePsi = appData.turbineInputPressureHpa * HPA_TO_PSI;
    in.nowMs = now;
    in.lastEec2RxMs = appData.lastEec2RxMs;
    in.lastEtc1RxMs = appData.lastEtc1RxMs;
    in.manualMode = manualMode;

    BrakeOutput out = exhaustBrakeStep(in, brakeConfig, brakeState, dt);

    appData.exhaustBrakeActive = out.active;
    if (out.active) {
        appData.actuatorDemandedPosition = out.vanePercent;
    }
    return out.active;
}
```

- [ ] **Step 3: Verify it compiles for the target**

Run: `pio run -e teensy41`
Expected: SUCCESS — `[SUCCESS]`. (Not called yet; this only confirms the glue compiles.)

- [ ] **Step 4: Commit**

```bash
git add src/control/exhaustBrakeController.h src/control/exhaustBrakeController.cpp
git commit -m "Add ExhaustBrakeController Arduino glue

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 7: Integrate into the orchestrator

**Files:**
- Modify: `src/domain/ovgt.cpp` (includes, `setup()`, `loop()`, `handleDebug()`)

- [ ] **Step 1: Add the include**

In `src/domain/ovgt.cpp`, add after the existing `#include "control/boostController.h"` line:

```cpp
#include "control/exhaustBrakeController.h"
```

- [ ] **Step 2: Initialize the controller**

In `src/domain/ovgt.cpp`, in `ovgt::setup()`, add immediately after the existing `BoostController::Initialize();` line:

```cpp
    ExhaustBrakeController::Initialize();
```

- [ ] **Step 3: Add the arbitration in the loop**

In `src/domain/ovgt.cpp`, in `ovgt::loop()`, replace these three lines:

```cpp
    BoostController::update();

    J1939::Loop();
```

with:

```cpp
    J1939::Loop();   // process RX first so brake/boost act on fresh CAN data

    bool braking = ExhaustBrakeController::update(manualMode);
    if (!braking) {
        BoostController::update();
    }
```

- [ ] **Step 4: Add brake state to the debug line**

In `src/domain/ovgt.cpp`, in `ovgt::handleDebug()`, change the `char buf[128];` declaration to:

```cpp
    char buf[160];
```

Then change the `snprintf(buf, ...)` format string and argument list to append the brake state. The format string gains a trailing ` Brk:%s` and the argument list gains a trailing argument. The final `snprintf` call becomes:

```cpp
    snprintf(buf, sizeof(buf), "BR:%s Boost:%.1fpsi BPR:%s Dem:%u%% Pos:%u%% TIP:%.1fpsi CIT:%dC CIP:%.1fpsi TIT:%dC Brk:%s",
        boBuf,
        (double)(boostGauge * 0.0145038f),
        brBuf,
        manualMode ? manualPwm : appData.actuatorDemandedPosition,
        appData.actuatorReportedPosition,
        (double)(appData.turbineInputPressureHpa * 0.0145038f),
        appData.compressorInputTempC,
        (double)(appData.compressorInputPressureHpaa * 0.0145038f),
        appData.turbineInletTempC,
        appData.exhaustBrakeActive ? "ON" : "off");
```

- [ ] **Step 5: Verify it compiles for the target**

Run: `pio run -e teensy41`
Expected: SUCCESS — `[SUCCESS]`.

- [ ] **Step 6: Commit**

```bash
git add src/domain/ovgt.cpp
git commit -m "Wire exhaust brake into orchestrator with actuator arbitration

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 8: Full verification

- [ ] **Step 1: Run the complete native test suite**

Run: `pio test -e native`
Expected: PASS — all three suites (`test_smoke`, `test_j1939_decode`, `test_exhaust_brake_logic`) report `0 Failures`.

- [ ] **Step 2: Build the firmware for the target**

Run: `pio run -e teensy41`
Expected: SUCCESS — `[SUCCESS]`, no warnings about the new code.

- [ ] **Step 3: Confirm git state is clean**

Run: `git status`
Expected: `nothing to commit, working tree clean` (all task commits already made).

---

## Post-implementation (on the truck — NOT part of coding)

These require the physical vehicle and are out of scope for this plan, but note them:
- Confirm 60 psi exhaust valve springs are installed before running `targetPsi >= ~55`.
- Tune `kp`/`ki` in `exhaustBrakeController.cpp` against real backpressure behavior.
- Verify ETC1/EEC2 are actually present on this truck's bus at the expected addresses.
