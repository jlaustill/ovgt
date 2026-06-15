# BPR=1.0 Boost Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the open-loop boost→vane lookup map with a closed-loop PI controller that targets a backpressure ratio (BPR = drive pressure ÷ boost) of 1.0, with a fixed spool position below a boost threshold.

**Architecture:** A new pure-logic module `boostBprLogic` (mirrors `exhaustBrakeLogic` — pure, native-unit-tested) computes the demanded vane position; the existing `BoostController` becomes thin glue that reads `appData`, calls it, and writes `actuatorDemandedPosition`. The main-loop arbitration in `ovgt.cpp` is unchanged (`BoostController::update()` already runs only when `!manualMode && !braking`).

**Tech Stack:** C++ (Teensy 4.1 / Arduino framework), PlatformIO, Unity test framework (`env:native`). Spec: `docs/plans/2026-06-14-boost-bpr-control-design.md`.

---

## File Structure

- **Create** `src/control/boostBprLogic.h` — types (`BoostInputs`, `BoostConfig`, `BoostState`) + `boostBprStep()` signature.
- **Create** `src/control/boostBprLogic.cpp` — the pure PI + spool logic. No Arduino dependencies.
- **Create** `test/test_boost_bpr_logic/test_boost_bpr_logic.cpp` — Unity native tests.
- **Modify** `platformio.ini` — add `boostBprLogic.cpp` to the `env:native` `build_src_filter`.
- **Replace** `src/control/boostController.h` — drop the map types, keep the `Initialize()/update()` interface.
- **Replace** `src/control/boostController.cpp` — glue calling `boostBprStep`.

**Vane convention (from `include/VaneConfig.h`):** `VANE_CLOSED_PERCENT = 0` (fully closed → max drive/boost), `VANE_OPEN_PERCENT = 88` (fully open → no restriction). Closing vanes raises drive pressure faster than boost, so **BPR below target → close; BPR above target → open** — the same error→closure sense as the brake.

---

## Task 1: BPR control pure logic + native tests (TDD)

**Files:**
- Create: `src/control/boostBprLogic.h`
- Create: `test/test_boost_bpr_logic/test_boost_bpr_logic.cpp`
- Modify: `platformio.ini` (native `build_src_filter`)
- Create: `src/control/boostBprLogic.cpp`

- [ ] **Step 1: Create the interface header**

Create `src/control/boostBprLogic.h`:

```cpp
#ifndef boostBprLogic_h
#define boostBprLogic_h

#include <stdint.h>

struct BoostInputs {
    float boostGaugePsi;   // (COP - CIP) in psi
    float tipGaugePsi;     // turbine inlet (drive) pressure, gauge psi
};

struct BoostConfig {
    float   bprTarget;          // 1.0
    float   boostMinPsi;        // below this -> spool region (BPR unreliable)
    uint8_t spoolPercent;       // fixed vane position while spooling
    uint8_t vaneClosedPercent;  // from VaneConfig.h
    uint8_t vaneOpenPercent;    // from VaneConfig.h
    float   kp;
    float   ki;
};

struct BoostState {
    float integralTerm;  // accumulated integral CONTRIBUTION, in vane-% units
    bool  wasSpooling;   // true while in the spool region; drives bumpless handoff
};

// Evaluate one boost control cycle and return the demanded vane position.
// Below cfg.boostMinPsi: hold cfg.spoolPercent (BPR is unreliable). Above it:
// PI loop driving BPR (= tipGaugePsi / boostGaugePsi) to cfg.bprTarget. Caller
// invokes this only when NOT in manual mode and NOT braking, so no mode/active
// flag is needed. Mutates state.
uint8_t boostBprStep(const BoostInputs &in, const BoostConfig &cfg,
                     BoostState &state, float dtSeconds);

#endif
```

- [ ] **Step 2: Write the failing tests**

Create `test/test_boost_bpr_logic/test_boost_bpr_logic.cpp`:

```cpp
#include <unity.h>
#include "control/boostBprLogic.h"

void setUp(void) {}
void tearDown(void) {}

// Matches the production config defaults (see boostController.cpp). NOTE: kp is
// large (~full vane travel per unit BPR error) because BPR error is a 0..1 ratio,
// not psi.
static BoostConfig makeConfig(void) {
    BoostConfig cfg;
    cfg.bprTarget = 1.0f;
    cfg.boostMinPsi = 2.0f;
    cfg.spoolPercent = 25;
    cfg.vaneClosedPercent = 0;
    cfg.vaneOpenPercent = 88;
    cfg.kp = 88.0f;
    cfg.ki = 20.0f;
    return cfg;
}

// Below the boost threshold, BPR is garbage (divide by ~0): hold the spool
// position and remember we were spooling.
void test_spool_region_holds_spool_position(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 1.0f;  // < boostMinPsi (2.0)
    in.tipGaugePsi = 0.5f;
    BoostState st = {0.0f, false};
    uint8_t vane = boostBprStep(in, cfg, st, 0.01f);
    TEST_ASSERT_EQUAL_UINT8(25, vane);
    TEST_ASSERT_TRUE(st.wasSpooling);
}

// BPR below target (drive too low) -> close vanes to raise drive pressure.
void test_bpr_below_target_closes(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 20.0f;
    in.tipGaugePsi = 10.0f;  // bpr = 0.5, error = 0.5
    BoostState st = {0.0f, false};
    uint8_t vane = boostBprStep(in, cfg, st, 0.01f);
    // integral = 20*0.5*0.01 = 0.1; closure = 88*0.5 + 0.1 = 44.1
    // vane = open(88) - 44.1 = 43.9 -> 44
    TEST_ASSERT_EQUAL_UINT8(44, vane);
    TEST_ASSERT_TRUE(vane < cfg.vaneOpenPercent);  // closing, not opening
}

// BPR above target (drive too high) -> open vanes (clamps fully open).
void test_bpr_above_target_opens(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 20.0f;
    in.tipGaugePsi = 30.0f;  // bpr = 1.5, error = -0.5
    BoostState st = {0.0f, false};
    uint8_t vane = boostBprStep(in, cfg, st, 0.01f);
    // negative error -> integral clamps to 0, closure = -44 -> vane clamps to open
    TEST_ASSERT_EQUAL_UINT8(88, vane);
}

// A bigger BPR deficit closes the vanes more.
void test_bigger_bpr_deficit_closes_more(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs small;
    small.boostGaugePsi = 20.0f;
    small.tipGaugePsi = 18.0f;  // bpr = 0.9, error = 0.1
    BoostState stSmall = {0.0f, false};
    uint8_t vaneSmall = boostBprStep(small, cfg, stSmall, 0.01f);

    BoostInputs big;
    big.boostGaugePsi = 20.0f;
    big.tipGaugePsi = 10.0f;   // bpr = 0.5, error = 0.5
    BoostState stBig = {0.0f, false};
    uint8_t vaneBig = boostBprStep(big, cfg, stBig, 0.01f);

    TEST_ASSERT_TRUE(vaneBig < vaneSmall);  // more deficit => more closure => lower %
}

// Handoff from spool to PI starts at the spool position, NOT a jump to fully open.
void test_bumpless_handoff_from_spool(void) {
    BoostConfig cfg = makeConfig();
    BoostState st = {0.0f, false};

    BoostInputs spool;
    spool.boostGaugePsi = 1.0f;  // spool region
    spool.tipGaugePsi = 20.0f;
    uint8_t vaneSpool = boostBprStep(spool, cfg, st, 0.01f);
    TEST_ASSERT_EQUAL_UINT8(25, vaneSpool);

    BoostInputs handoff;
    handoff.boostGaugePsi = 20.0f;  // now above threshold
    handoff.tipGaugePsi = 20.0f;    // bpr = 1.0, error = 0
    uint8_t vaneHandoff = boostBprStep(handoff, cfg, st, 0.01f);
    // seeded integral = open(88) - spool(25) = 63; closure = 88*0 + 63 = 63
    // vane = 88 - 63 = 25  (continues from spool, no jump to 88)
    TEST_ASSERT_EQUAL_UINT8(25, vaneHandoff);
    TEST_ASSERT_TRUE(vaneHandoff < cfg.vaneOpenPercent);
}

// Persistent large deficit: integral winds up to full travel, vanes fully closed.
void test_integral_antiwindup_clamps_closed(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 20.0f;
    in.tipGaugePsi = 0.0f;  // bpr = 0, error = 1.0 (persistent)
    BoostState st = {0.0f, false};
    uint8_t vane = 0;
    for (int i = 0; i < 1000; i++) {
        vane = boostBprStep(in, cfg, st, 0.01f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 88.0f, st.integralTerm);  // clamped at full travel
    TEST_ASSERT_EQUAL_UINT8(0, vane);                          // fully closed
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_spool_region_holds_spool_position);
    RUN_TEST(test_bpr_below_target_closes);
    RUN_TEST(test_bpr_above_target_opens);
    RUN_TEST(test_bigger_bpr_deficit_closes_more);
    RUN_TEST(test_bumpless_handoff_from_spool);
    RUN_TEST(test_integral_antiwindup_clamps_closed);
    return UNITY_END();
}
```

- [ ] **Step 3: Add the new module to the native build filter**

In `platformio.ini`, under `[env:native]`, change the `build_src_filter` line to include `boostBprLogic.cpp`:

```ini
build_src_filter = -<*> +<control/exhaustBrakeLogic.cpp> +<control/boostBprLogic.cpp> +<sensors/j1939Decode.cpp>
```

- [ ] **Step 4: Run the tests to verify they FAIL**

Run: `pio test -e native -f test_boost_bpr_logic`
Expected: build/link FAILS with `undefined reference to 'boostBprStep(...)'` (header exists, implementation does not yet).

- [ ] **Step 5: Implement the logic**

Create `src/control/boostBprLogic.cpp`:

```cpp
#include "boostBprLogic.h"

static uint8_t clampVane(float vane, uint8_t lo, uint8_t hi) {
    if (vane < (float)lo) return lo;
    if (vane > (float)hi) return hi;
    return (uint8_t)(vane + 0.5f);
}

// Vane convention: vaneClosedPercent (0) = fully closed = max drive pressure /
// boost; vaneOpenPercent (88) = fully open = no restriction. BPR = drive / boost.
// Closing vanes raises drive faster than boost, so BPR rises. To hold BPR at
// target: BPR below target -> close (raise drive); BPR above target -> open.
uint8_t boostBprStep(const BoostInputs &in, const BoostConfig &cfg,
                     BoostState &state, float dtSeconds) {
    // 1. Spool region. BPR = drive/boost is unreliable at low boost (divide by
    //    ~0), and a runaway BPR would fling the vanes open — the opposite of what
    //    spool needs. Hold a fixed spool position and flag that we were spooling.
    if (in.boostGaugePsi < cfg.boostMinPsi) {
        state.wasSpooling = true;
        return clampVane((float)cfg.spoolPercent,
                         cfg.vaneClosedPercent, cfg.vaneOpenPercent);
    }

    float travel = (float)cfg.vaneOpenPercent - (float)cfg.vaneClosedPercent;

    // 2. Bumpless handoff. On the first PI cycle after spooling, seed the integral
    //    so the PI's output starts AT the spool position instead of jumping to
    //    fully open (vane = open - 0). Then the loop adjusts from there.
    if (state.wasSpooling) {
        state.integralTerm = (float)cfg.vaneOpenPercent - (float)cfg.spoolPercent;
        state.wasSpooling = false;
    }

    // 3. PI loop on BPR. No derivative term: BPR is a noisy ratio. Positive error
    //    (BPR below target) means we need MORE drive pressure, i.e. MORE closure,
    //    i.e. a LOWER vane position. Closure adds up and subtracts from open.
    float bpr = in.tipGaugePsi / in.boostGaugePsi;
    float error = cfg.bprTarget - bpr;
    state.integralTerm += cfg.ki * error * dtSeconds;
    if (state.integralTerm < 0.0f) {
        state.integralTerm = 0.0f;
    }
    if (state.integralTerm > travel) {
        state.integralTerm = travel;  // anti-windup: cannot exceed full travel
    }
    float closure = cfg.kp * error + state.integralTerm;
    float vane = (float)cfg.vaneOpenPercent - closure;
    return clampVane(vane, cfg.vaneClosedPercent, cfg.vaneOpenPercent);
}
```

- [ ] **Step 6: Run the tests to verify they PASS**

Run: `pio test -e native -f test_boost_bpr_logic`
Expected: PASS (6 tests).

Then run the full native suite to confirm no regressions:
Run: `pio test -e native`
Expected: PASS (boost, exhaust brake, j1939, smoke suites all green).

- [ ] **Step 7: Commit**

```bash
git add src/control/boostBprLogic.h src/control/boostBprLogic.cpp test/test_boost_bpr_logic/test_boost_bpr_logic.cpp platformio.ini
git commit -m "$(cat <<'EOF'
Add BPR=1.0 boost control logic with native tests

Pure PI controller targeting BPR (drive/boost) = 1.0, with a fixed spool
position below a boost threshold and bumpless handoff into the PI. Mirrors
exhaustBrakeLogic; native Unity tests cover spool, direction, gradient,
handoff, and anti-windup.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: Rewrite BoostController glue to use the BPR loop

**Files:**
- Modify: `src/control/boostController.h` (full replace)
- Modify: `src/control/boostController.cpp` (full replace)

- [ ] **Step 1: Slim the header (remove the lookup-map types)**

Replace the entire contents of `src/control/boostController.h` with:

```cpp
#ifndef BoostController_h
#define BoostController_h

class BoostController {
    public:
        static void Initialize();
        static void update();
};

#endif
```

- [ ] **Step 2: Rewrite the glue**

Replace the entire contents of `src/control/boostController.cpp` with:

```cpp
#include <Arduino.h>
#include "boostController.h"
#include "boostBprLogic.h"
#include "AppData.h"
#include "VaneConfig.h"

static const float HPA_TO_PSI = 0.0145038f;

// --- Tunable configuration -------------------------------------------------
// Closed-loop vane control targeting BPR (drive pressure / boost) = 1.0. Below
// boostMinPsi, BPR is unreliable (divide by ~0) so hold a fixed spool position.
// Vane limits come from VaneConfig.h. kp/ki/spoolPercent/boostMinPsi are starting
// estimates — tune on the truck. NOTE: BPR error is a 0..1 ratio, so kp is large
// (~full vane travel per unit error), unlike the brake's psi-scale kp.
// No boost ceiling: this targets BPR only (deliberate — see design doc).
static BoostConfig boostConfig = {
    1.0f,                // bprTarget
    2.0f,                // boostMinPsi (spool/BPR boundary)
    25,                  // spoolPercent (fixed vane position while spooling)
    VANE_CLOSED_PERCENT, // vaneClosedPercent (max drive / boost)
    VANE_OPEN_PERCENT,   // vaneOpenPercent (relief / mechanical open limit)
    88.0f,               // kp (%vane per unit BPR error)
    20.0f                // ki (%vane per (BPR error * s))
};

static BoostState boostState = {0.0f, true};
static uint32_t lastUpdateMs = 0;

void BoostController::Initialize() {
    boostState.integralTerm = 0.0f;
    boostState.wasSpooling = true;
    lastUpdateMs = millis();
    Serial.println("BoostController initialized");
}

void BoostController::update() {
    uint32_t now = millis();
    float dt = (now - lastUpdateMs) / 1000.0f;
    lastUpdateMs = now;
    if (dt <= 0.0f) dt = 0.01f;

    int16_t boostGaugeHpa = (int16_t)appData.compressorOutputPressureHpaa
                          - (int16_t)appData.compressorInputPressureHpaa;
    if (boostGaugeHpa < 0) boostGaugeHpa = 0;

    BoostInputs in;
    in.boostGaugePsi = boostGaugeHpa * HPA_TO_PSI;
    in.tipGaugePsi = appData.turbineInputPressureHpa * HPA_TO_PSI;

    appData.actuatorDemandedPosition = boostBprStep(in, boostConfig, boostState, dt);
}
```

- [ ] **Step 3: Build for the target to verify it compiles**

Run: `pio run -e teensy41`
Expected: SUCCESS (`boostBprLogic.cpp` and the rewritten glue compile and link for Teensy 4.1; the removed `pressureMap`/`interpolate` cause no dangling references).

- [ ] **Step 4: Commit**

```bash
git add src/control/boostController.h src/control/boostController.cpp
git commit -m "$(cat <<'EOF'
Switch BoostController to BPR=1.0 closed loop (remove lookup map)

BoostController is now thin glue: reads boost (COP-CIP) and drive pressure
(TIP) from appData and calls boostBprStep. Removes the open-loop pressure
lookup map and interpolation. Arbitration in ovgt.cpp is unchanged.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
EOF
)"
```

---

## Verification Summary

After all tasks:
- `pio test -e native` — all suites green (new `test_boost_bpr_logic` + existing brake/j1939/smoke).
- `pio run -e teensy41` — firmware builds with the BPR loop (clock stays at 600 MHz; the thermal fix is relocating the controller away from the turbo, not underclocking).
- On-truck (separate): confirm vanes hold BPR≈1.0 above the boost threshold and the spool→PI handoff is smooth; tune `kp`/`ki`/`boostMinPsi`/`spoolPercent` in `boostController.cpp`.

## Out of Scope (per design doc)

- Boost-ceiling over-boost guard (deliberately omitted).
- Extracting a shared `piStep` primitive used by both brake and boost (do once both loops are tuned).
- Converting boost control to C-Next (later migration step).
- Physical relocation of the controller away from the turbo (hardware task — this is the thermal fix; clock stays at 600 MHz).
