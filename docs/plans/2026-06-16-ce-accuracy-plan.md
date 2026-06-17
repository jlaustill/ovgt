# CE Accuracy Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make compressor-efficiency (CE) trustworthy and add an in-firmware probe time-constant (τ) measurement, in software only.

**Architecture:** Store CIT/COT as `float` (resolution); add a pure `cotSettle` module that, fed each fresh COT sample (~10 Hz), produces a CE "settled" trust-flag and — using boost-settle as a fast reference — a τ + settling-time measurement after each clean step. Glue wires it into `ovgt.cpp` (CE `~` marker + a `COT settle:` report line).

**Tech Stack:** C++ (Teensy 4.1 / Arduino), PlatformIO, Unity (`env:native`). Spec: `docs/plans/2026-06-16-ce-accuracy-design.md`.

---

## File Structure

- **Modify** `include/AppData.h` — `compressorInputTempC` / `compressorOutputTempC` → `float`.
- **Modify** `src/sensors/adcSensors.cpp` — store CIT as float (drop cast).
- **Modify** `src/sensors/cotSensor.{h,cpp}` — store COT as float; `update()` returns `bool` (fresh sample).
- **Create** `src/sensors/cotSettle.{h,cpp}` — pure settled-flag + τ measurement.
- **Create** `test/test_cot_settle/test_cot_settle.cpp` — native Unity tests.
- **Modify** `platformio.ini` — add `cotSettle.cpp` to the native `build_src_filter`.
- **Modify** `src/domain/ovgt.cpp` — float temp display, feed `cotSettle`, CE `~` marker, τ report line.

Vane/control code is untouched. `j1939.cpp` does **not** reference CIT/COT (verified) — no change.

---

## Task 1: CIT/COT → float resolution

**Files:**
- Modify: `include/AppData.h:14,20`
- Modify: `src/sensors/adcSensors.cpp` (processResult case 2)
- Modify: `src/sensors/cotSensor.cpp` (COT write)
- Modify: `src/domain/ovgt.cpp` (debug format)

This is a type/format change (no new logic), verified by the existing suite + builds.

- [ ] **Step 1: Widen the AppData fields to float**

In `include/AppData.h`, change:
```cpp
    int16_t compressorInputTempC;
```
to
```cpp
    float compressorInputTempC;
```
and
```cpp
    int16_t compressorOutputTempC;
```
to
```cpp
    float compressorOutputTempC;
```

- [ ] **Step 2: Store CIT as float**

In `src/sensors/adcSensors.cpp`, processResult case 2, change:
```cpp
            appData.compressorInputTempC = (int16_t)steinhartHart(resistance);
```
to
```cpp
            appData.compressorInputTempC = steinhartHart(resistance);
```

- [ ] **Step 3: Store COT as float**

In `src/sensors/cotSensor.cpp`, change:
```cpp
    appData.compressorOutputTempC = (int16_t)tc.readThermocoupleTemperature();
```
to
```cpp
    appData.compressorOutputTempC = tc.readThermocoupleTemperature();
```

- [ ] **Step 4: Print CIT/COT with one decimal**

In `src/domain/ovgt.cpp` `handleDebug()`, the format string currently contains `CIT:%dC COT:%dC` — change that substring to `CIT:%.1fC COT:%.1fC`. Then change the two matching args from:
```cpp
        appData.compressorInputTempC,
        appData.compressorOutputTempC,
```
to
```cpp
        (double)appData.compressorInputTempC,
        (double)appData.compressorOutputTempC,
```
(`printf` `%f` requires `double`; the surrounding args/format are otherwise unchanged.)

- [ ] **Step 5: Run native tests (nothing should break)**

Run: `pio test -e native`
Expected: PASS — all suites, including `test_compressor_efficiency` (it already takes floats).

- [ ] **Step 6: Build firmware**

Run: `pio run -e teensy41`
Expected: SUCCESS.

- [ ] **Step 7: Commit**

```bash
git add include/AppData.h src/sensors/adcSensors.cpp src/sensors/cotSensor.cpp src/domain/ovgt.cpp
git commit -m "$(cat <<'EOF'
Store CIT/COT as float (sub-degree resolution for CE)

CE divides by (COT-CIT); whole-degree storage was the noise floor. Fields ->
float, sensor writes drop the int cast, debug prints %.1f. j1939 doesn't TX
these, control loops don't read them.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: `cotSettle` pure module (settled flag + τ measurement)

**Files:**
- Create: `src/sensors/cotSettle.h`
- Create: `test/test_cot_settle/test_cot_settle.cpp`
- Modify: `platformio.ini` (native `build_src_filter`)
- Create: `src/sensors/cotSettle.cpp`

- [ ] **Step 1: Create the interface header**

Create `src/sensors/cotSettle.h`:
```cpp
#ifndef cotSettle_h
#define cotSettle_h

#include <stdint.h>

struct CotSettleConfig {
    float    cotSlopeFlatCperS;     // |dCOT/dt| below this => COT "not moving"
    float    boostSlopeFlatPsiPerS; // |dBoost/dt| below this => boost "steady"
    float    settledSeconds;        // sustained-flat duration for the CE settled flag
    float    minStepC;              // min |COT change| to accept a tau measurement
    uint16_t maxBufferSamples;      // cap on samples buffered during one measurement
};

struct CotSettleResult {
    bool  settled;          // CE-trust flag: COT + boost both flat, sustained
    bool  measurementReady; // true the one sample a tau measurement completes
    float tauSeconds;       // valid when measurementReady (63% time constant)
    float settleSeconds;    // valid when measurementReady (time to slope-flat)
    float stepC;            // signed COT change (valid when measurementReady)
};

// Max samples buffered during one measurement. ~512 @ ~10 Hz covers ~51 s.
static const uint16_t COT_SETTLE_BUFFER = 512;

struct CotSettleState {
    bool     initialized;
    float    lastCotC;
    float    lastBoostPsi;
    float    settledTimer;   // accumulated flat time (CE settled flag)
    bool     armed;          // currently timing a COT settle
    bool     boostWasFlat;   // previous-sample boost-flat (edge detect)
    bool     cotMoved;       // COT left the flat band during this measurement
    float    measElapsed;    // time since arm
    float    cotStart;       // COT at arm
    uint16_t count;          // samples buffered this measurement
    float    bufElapsed[COT_SETTLE_BUFFER];
    float    bufCot[COT_SETTLE_BUFFER];
};

void cotSettleInit(CotSettleState &state);

// Feed one fresh COT sample (~10 Hz). Mutates state; returns the settled flag and
// (occasionally) a completed tau measurement.
CotSettleResult cotSettleStep(CotSettleState &state, const CotSettleConfig &cfg,
                              float cotC, float boostPsi, float dtSeconds);

#endif
```

- [ ] **Step 2: Write the failing tests**

Create `test/test_cot_settle/test_cot_settle.cpp`:
```cpp
#include <unity.h>
#include <math.h>
#include "sensors/cotSettle.h"

void setUp(void) {}
void tearDown(void) {}

static CotSettleConfig makeConfig(void) {
    CotSettleConfig cfg;
    cfg.cotSlopeFlatCperS = 0.3f;
    cfg.boostSlopeFlatPsiPerS = 0.5f;
    cfg.settledSeconds = 2.0f;
    cfg.minStepC = 3.0f;
    cfg.maxBufferSamples = COT_SETTLE_BUFFER;
    return cfg;
}

// Sustained flat COT + flat boost -> settled after settledSeconds.
void test_settled_flag_after_sustained_flat(void) {
    CotSettleConfig cfg = makeConfig();
    CotSettleState st; cotSettleInit(st);
    cotSettleStep(st, cfg, 50.0f, 5.0f, 0.1f);  // seed
    CotSettleResult r = {false,false,0,0,0};
    for (int k = 0; k < 25; k++) r = cotSettleStep(st, cfg, 50.0f, 5.0f, 0.1f);
    TEST_ASSERT_TRUE(r.settled);  // 25*0.1 = 2.5s >= 2.0s
}

// A COT spike after being settled clears the flag.
void test_settled_resets_on_spike(void) {
    CotSettleConfig cfg = makeConfig();
    CotSettleState st; cotSettleInit(st);
    cotSettleStep(st, cfg, 50.0f, 5.0f, 0.1f);
    for (int k = 0; k < 25; k++) cotSettleStep(st, cfg, 50.0f, 5.0f, 0.1f);
    CotSettleResult r = cotSettleStep(st, cfg, 60.0f, 5.0f, 0.1f);  // +100 C/s spike
    TEST_ASSERT_FALSE(r.settled);
}

// Boost flat, COT follows a known first-order step -> tau extracted ~correctly.
void test_tau_extraction_first_order(void) {
    CotSettleConfig cfg = makeConfig();
    CotSettleState st; cotSettleInit(st);
    const float dt = 0.1f, tau = 0.5f, c0 = 20.0f, c1 = 120.0f;
    cotSettleStep(st, cfg, c0, 0.0f, dt);   // seed
    cotSettleStep(st, cfg, c0, 2.0f, dt);   // boost moving (slope 20) -> not flat
    CotSettleResult r = {false,false,0,0,0};
    float t = 0.0f;
    for (int k = 0; k < 200 && !r.measurementReady; k++) {
        float cot = c1 - (c1 - c0) * expf(-t / tau);  // arm at t=0 (boost now flat)
        r = cotSettleStep(st, cfg, cot, 2.0f, dt);
        t += dt;
    }
    TEST_ASSERT_TRUE(r.measurementReady);
    TEST_ASSERT_FLOAT_WITHIN(0.08f, 0.5f, r.tauSeconds);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 100.0f, r.stepC);
}

// Boost never settles -> never arms -> no measurement.
void test_no_measurement_when_boost_moving(void) {
    CotSettleConfig cfg = makeConfig();
    CotSettleState st; cotSettleInit(st);
    cotSettleStep(st, cfg, 20.0f, 0.0f, 0.1f);  // seed
    bool any = false;
    for (int k = 1; k < 40; k++) {
        float cot = 120.0f - 100.0f * expf(-(k * 0.1f) / 0.5f);
        float boost = (float)k * 2.0f;  // always rising -> slope 20, never flat
        CotSettleResult r = cotSettleStep(st, cfg, cot, boost, 0.1f);
        any = any || r.measurementReady;
    }
    TEST_ASSERT_FALSE(any);
}

// A clean settle but with a tiny COT step (< minStepC) is discarded.
void test_small_step_rejected(void) {
    CotSettleConfig cfg = makeConfig();
    CotSettleState st; cotSettleInit(st);
    cotSettleStep(st, cfg, 20.0f, 0.0f, 0.1f);  // seed
    cotSettleStep(st, cfg, 20.0f, 2.0f, 0.1f);  // boost moving
    // boost flat; COT nudges +1 C then holds (well under minStepC=3)
    float cots[] = {20.0f, 20.6f, 21.0f, 21.0f, 21.0f, 21.0f, 21.0f, 21.0f};
    bool any = false;
    for (float c : cots) {
        CotSettleResult r = cotSettleStep(st, cfg, c, 2.0f, 0.1f);
        any = any || r.measurementReady;
    }
    TEST_ASSERT_FALSE(any);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_settled_flag_after_sustained_flat);
    RUN_TEST(test_settled_resets_on_spike);
    RUN_TEST(test_tau_extraction_first_order);
    RUN_TEST(test_no_measurement_when_boost_moving);
    RUN_TEST(test_small_step_rejected);
    return UNITY_END();
}
```

- [ ] **Step 3: Add the module to the native build filter**

In `platformio.ini`, under `[env:native]`, change the `build_src_filter` line to also include `cotSettle.cpp`:
```ini
build_src_filter = -<*> +<control/exhaustBrakeLogic.cpp> +<control/boostBprLogic.cpp> +<sensors/j1939Decode.cpp> +<sensors/compressorEfficiency.cpp> +<sensors/cotSettle.cpp>
```

- [ ] **Step 4: Run tests to verify they FAIL**

Run: `pio test -e native -f test_cot_settle`
Expected: build/link FAILS — `undefined reference to cotSettleInit / cotSettleStep`.

- [ ] **Step 5: Implement the module**

Create `src/sensors/cotSettle.cpp`:
```cpp
#include "cotSettle.h"
#include <math.h>

void cotSettleInit(CotSettleState &s) {
    s.initialized = false;
    s.lastCotC = 0.0f;
    s.lastBoostPsi = 0.0f;
    s.settledTimer = 0.0f;
    s.armed = false;
    s.boostWasFlat = false;
    s.cotMoved = false;
    s.measElapsed = 0.0f;
    s.cotStart = 0.0f;
    s.count = 0;
}

// First crossing of the 63.2% point, linearly interpolated -> time constant.
static float extractTau(const CotSettleState &s, float cotStart, float cotEnd) {
    float step = cotEnd - cotStart;
    float target = cotStart + 0.632f * step;
    float tau = s.measElapsed;  // fallback
    for (uint16_t i = 0; i < s.count; i++) {
        bool reached = (step > 0.0f) ? (s.bufCot[i] >= target) : (s.bufCot[i] <= target);
        if (reached) {
            if (i == 0) {
                tau = s.bufElapsed[0];
            } else {
                float c0 = s.bufCot[i - 1], c1 = s.bufCot[i];
                float t0 = s.bufElapsed[i - 1], t1 = s.bufElapsed[i];
                float frac = (c1 != c0) ? (target - c0) / (c1 - c0) : 0.0f;
                tau = t0 + frac * (t1 - t0);
            }
            break;
        }
    }
    return tau;
}

CotSettleResult cotSettleStep(CotSettleState &s, const CotSettleConfig &cfg,
                              float cotC, float boostPsi, float dtSeconds) {
    CotSettleResult r = {false, false, 0.0f, 0.0f, 0.0f};

    if (!s.initialized) {              // seed sample: no slope available yet
        s.initialized = true;
        s.lastCotC = cotC;
        s.lastBoostPsi = boostPsi;
        s.boostWasFlat = false;
        return r;
    }

    if (dtSeconds <= 0.0f) dtSeconds = 0.0001f;
    float cotSlope = (cotC - s.lastCotC) / dtSeconds;
    float boostSlope = (boostPsi - s.lastBoostPsi) / dtSeconds;
    bool cotFlat = fabsf(cotSlope) < cfg.cotSlopeFlatCperS;
    bool boostFlat = fabsf(boostSlope) < cfg.boostSlopeFlatPsiPerS;

    // CE settled flag: both flat, sustained.
    if (cotFlat && boostFlat) {
        s.settledTimer += dtSeconds;
    } else {
        s.settledTimer = 0.0f;
    }
    r.settled = (s.settledTimer >= cfg.settledSeconds);

    // tau measurement event machine (boost-settle is the fast reference).
    if (!s.armed) {
        if (boostFlat && !s.boostWasFlat) {   // boost just went flat -> arm
            s.armed = true;
            s.cotMoved = false;
            s.measElapsed = 0.0f;
            s.cotStart = cotC;
            s.bufElapsed[0] = 0.0f;
            s.bufCot[0] = cotC;
            s.count = 1;
        }
    } else {
        s.measElapsed += dtSeconds;
        if (!boostFlat) {
            s.armed = false;                  // boost moved again -> not a clean step
        } else {
            if (s.count < cfg.maxBufferSamples && s.count < COT_SETTLE_BUFFER) {
                s.bufElapsed[s.count] = s.measElapsed;
                s.bufCot[s.count] = cotC;
                s.count++;
            } else {
                s.armed = false;              // buffer full -> abort
            }
            if (!cotFlat) s.cotMoved = true;
            if (s.armed && cotFlat && s.cotMoved) {
                float step = cotC - s.cotStart;
                if (fabsf(step) >= cfg.minStepC) {
                    r.measurementReady = true;
                    r.tauSeconds = extractTau(s, s.cotStart, cotC);
                    r.settleSeconds = s.measElapsed;
                    r.stepC = step;
                }
                s.armed = false;              // completed or discarded (too small)
            }
        }
    }

    s.boostWasFlat = boostFlat;
    s.lastCotC = cotC;
    s.lastBoostPsi = boostPsi;
    return r;
}
```

- [ ] **Step 6: Run tests to verify they PASS**

Run: `pio test -e native -f test_cot_settle`
Expected: PASS (5 tests).
Then full suite: `pio test -e native`
Expected: PASS (all suites).

- [ ] **Step 7: Commit**

```bash
git add src/sensors/cotSettle.h src/sensors/cotSettle.cpp test/test_cot_settle/ platformio.ini
git commit -m "$(cat <<'EOF'
Add cotSettle: CE settled-flag + probe tau measurement

Pure, native-tested. Tracks COT/boost slope; emits a sustained-flat 'settled'
flag for CE trust, and (arming on boost-settle as a fast reference) measures the
COT settling time + 63% time constant after each clean step. Tests cover the
flag, tau extraction from a synthetic first-order step, and the abort paths.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: Wire `cotSettle` into the firmware

**Files:**
- Modify: `src/sensors/cotSensor.h` (return type)
- Modify: `src/sensors/cotSensor.cpp` (return bool)
- Modify: `src/domain/ovgt.cpp` (includes, statics, setup, loop feed, CE marker)

- [ ] **Step 1: `CotSensor::update()` returns "fresh sample"**

In `src/sensors/cotSensor.h`, change:
```cpp
        static void update();
```
to
```cpp
        static bool update();
```

In `src/sensors/cotSensor.cpp`, change the whole `update()` to return `bool` (true only when it stored a fresh, fault-free reading):
```cpp
bool CotSensor::update() {
    if (digitalRead(DRDY_PIN)) return false;  // no fresh conversion
    uint8_t fault = tc.readFault();
    if (fault) {
        static uint32_t lastFaultPrint = 0;
        if (millis() - lastFaultPrint > 1000) {
            lastFaultPrint = millis();
            Serial.print("COT fault=0x");
            Serial.println(fault, HEX);
        }
        return false;                          // faulted -> not a usable sample
    }
    appData.compressorOutputTempC = tc.readThermocoupleTemperature();
    return true;
}
```

- [ ] **Step 2: Add the include + module state to ovgt.cpp**

In `src/domain/ovgt.cpp`, after the existing `#include "sensors/compressorEfficiency.h"` line, add:
```cpp
#include "sensors/cotSettle.h"
```
Then, near the other file-scope objects (after `AppData appData;`), add:
```cpp
static CotSettleState cotSettleState;
static CotSettleConfig cotSettleCfg = {
    0.3f,   // cotSlopeFlatCperS
    0.5f,   // boostSlopeFlatPsiPerS
    2.0f,   // settledSeconds
    3.0f,   // minStepC
    512     // maxBufferSamples
};
static bool ceSettled = false;
static elapsedMicros cotSampleDt;
```

- [ ] **Step 3: Initialize the detector in setup()**

In `src/domain/ovgt.cpp` `setup()`, immediately after `BoostController::Initialize();`, add:
```cpp
    cotSettleInit(cotSettleState);
```

- [ ] **Step 4: Feed cotSettle on each fresh COT sample**

In `src/domain/ovgt.cpp` `loop()`, replace the single line:
```cpp
    CotSensor::update();
```
with:
```cpp
    if (CotSensor::update()) {
        float cotDt = cotSampleDt / 1000000.0f;
        cotSampleDt = 0;
        int16_t boostHpa = (int16_t)appData.compressorOutputPressureHpaa
                         - (int16_t)appData.compressorInputPressureHpaa;
        if (boostHpa < 0) boostHpa = 0;
        CotSettleResult ceRes = cotSettleStep(cotSettleState, cotSettleCfg,
                                              appData.compressorOutputTempC,
                                              boostHpa * 0.0145038f, cotDt);
        ceSettled = ceRes.settled;
        if (ceRes.measurementReady) {
            char sbuf[64];
            snprintf(sbuf, sizeof(sbuf),
                     "COT settle: tau=%.1fs t_settle=%.1fs dT=%+.1fC",
                     (double)ceRes.tauSeconds, (double)ceRes.settleSeconds,
                     (double)ceRes.stepC);
            Serial.println(sbuf);
        }
    }
```

- [ ] **Step 5: Add the `~` un-settled marker to CE**

In `src/domain/ovgt.cpp` `handleDebug()`, change the CE formatting:
```cpp
    if (efficiency >= 0.0f) {
        snprintf(ceBuf, sizeof(ceBuf), "%.0f%%", (double)(efficiency * 100.0f));
    } else {
        snprintf(ceBuf, sizeof(ceBuf), "--");
    }
```
to:
```cpp
    if (efficiency >= 0.0f) {
        snprintf(ceBuf, sizeof(ceBuf), "%.0f%%%s",
                 (double)(efficiency * 100.0f), ceSettled ? "" : "~");
    } else {
        snprintf(ceBuf, sizeof(ceBuf), "--");
    }
```

- [ ] **Step 6: Build firmware**

Run: `pio run -e teensy41`
Expected: SUCCESS.

- [ ] **Step 7: Run native suite (no regressions)**

Run: `pio test -e native`
Expected: PASS (all suites).

- [ ] **Step 8: Commit**

```bash
git add src/sensors/cotSensor.h src/sensors/cotSensor.cpp src/domain/ovgt.cpp
git commit -m "$(cat <<'EOF'
Wire cotSettle into telemetry: CE ~ marker + COT settle report

CotSensor::update() now returns whether it wrote a fresh sample; the loop feeds
cotSettle at the COT conversion rate. CE prints a trailing ~ when unsettled
(transient/untrustworthy), and each clean tip-in->cruise prints a
'COT settle: tau=.. t_settle=.. dT=..' line for probe characterization.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
EOF
)"
```

---

## Verification Summary

After all tasks:
- `pio test -e native` — all suites green (adds `test_cot_settle`, 5 cases).
- `pio run -e teensy41` — builds.
- On-truck: CE shows `~` on tip-ins, clean on cruise; a tip-in→cruise prints a `COT settle:` line. Read the sheathed probe's τ; swap the exposed junk-pile probe and compare.

## Out of Scope (per design)
- Faster-probe hardware swap (B1/B2) — documented in the design, separate effort.
- Software lag-compensation (Approach C) — future; the τ this produces would feed it.
- Mongo logger integration — consumes `ceSettled` + float temps when it lands.
