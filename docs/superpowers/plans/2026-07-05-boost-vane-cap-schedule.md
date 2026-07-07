# Boost Vane Open-Cap Schedule — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the flat vane open-cap (`vaneOpenCapPercent = 55`) with a boost→open-cap schedule so the BPR PI loop can no longer slam the vane wide open under load.

**Architecture:** A pure, native-tested interpolator (`vaneOpenCapForBoost`) maps boost psi to an open-cap % from a tweakable table; `boostBprStep`'s final clamp uses it instead of the scalar cap. Schedule is wired as nullable trailing fields on `BoostConfig` (null → existing flat-cap behavior), so validated tests stay green. The active cap is surfaced as telemetry `vane_cap`.

**Tech Stack:** C++ (Teensy 4.1 / Arduino / PlatformIO), Unity native tests, Node/TypeScript host tool (vitest).

## Global Constraints

- Vane convention: `0%` = fully closed (max drive/boost), `88%` = fully open. Higher % = more open.
- Tuning is **compile-time only** — edit source + reflash. NO runtime/serial setters (driving hazard).
- Floor stays `spoolPercent = 22` (validated; 18 spiked EGT). This change touches the **cap only**.
- `boostBprStep` is pure and native-tested (`test/test_boost_bpr_logic/`). Keep it pure.
- No automatic EGT escape this iteration (driver watches TIT + `act_load` live).
- Production schedule points: `{5,22} {10,25} {15,27} {20,29} {25,31}` (boost psi → cap %).
- New host telemetry fields are optional (`?`) — older logs lack them.
- Branch: `feature/boost-vane-cap-schedule`. Build: `pio run -e teensy41`. Native tests: `pio test -e native -f test_boost_bpr_logic`.

---

### Task 1: Pure `vaneOpenCapForBoost` interpolator + config/state fields

**Files:**
- Modify: `src/control/boostBprLogic.h` (add `VaneCapPoint`, `BoostConfig` schedule fields, `BoostState.lastVaneCap`, declare function)
- Modify: `src/control/boostBprLogic.cpp` (implement function)
- Test: `test/test_boost_bpr_logic/test_boost_bpr_logic.cpp` (null-init new config fields in `makeConfig`; add interpolator tests)

**Interfaces:**
- Produces: `struct VaneCapPoint { float boostPsi; uint8_t capPercent; };`
- Produces: `uint8_t vaneOpenCapForBoost(float boostPsi, const BoostConfig &cfg);`
- Produces: `BoostConfig` gains `const VaneCapPoint *vaneCapSchedule;` and `uint8_t vaneCapScheduleLen;` (trailing).
- Produces: `BoostState` gains trailing `uint8_t lastVaneCap;`

- [ ] **Step 1: Add the struct/field declarations to the header**

In `src/control/boostBprLogic.h`, add above `struct BoostConfig`:
```c
struct VaneCapPoint {
    float   boostPsi;     // schedule breakpoint (ascending)
    uint8_t capPercent;   // open-cap at/above this boost (until the next point)
};
```
Append to `struct BoostConfig` (after `vaneOpenCapPercent`):
```c
    // Boost -> open-cap schedule (ascending boostPsi). NULL / len 0 => use the flat
    // vaneOpenCapPercent above. Tightens open authority under load so the PI cannot
    // slam the vane wide open. Floor stays spoolPercent.
    const VaneCapPoint *vaneCapSchedule;
    uint8_t             vaneCapScheduleLen;
```
Append to `struct BoostState` (after `inPiRegion`):
```c
    uint8_t lastVaneCap;  // open-cap used on the last step (for telemetry readout)
```
Declare after the `boostBprStep` prototype:
```c
// Boost (gauge psi) -> vane open-cap %, linear between ascending schedule points,
// flat outside the ends. Falls back to cfg.vaneOpenCapPercent when no schedule is
// wired (cfg.vaneCapSchedule == NULL or cfg.vaneCapScheduleLen == 0).
uint8_t vaneOpenCapForBoost(float boostPsi, const BoostConfig &cfg);
```

- [ ] **Step 2: Null-init the new config fields in the test helper**

In `test/test_boost_bpr_logic/test_boost_bpr_logic.cpp`, inside `makeConfig()`, before `return cfg;`, add:
```c
    cfg.vaneCapSchedule = nullptr;   // existing tests: flat-cap path
    cfg.vaneCapScheduleLen = 0;
```
(`BoostConfig cfg;` is default-initialized, so the new pointer would be garbage otherwise.)

- [ ] **Step 3: Write the failing interpolator tests**

Append to `test/test_boost_bpr_logic/test_boost_bpr_logic.cpp` (and add matching `RUN_TEST` lines in `main`):
```c
static const VaneCapPoint kSchedule[] = {
    {5.0f, 22}, {10.0f, 25}, {15.0f, 27}, {20.0f, 29}, {25.0f, 31},
};

void test_vane_cap_null_schedule_falls_back_to_flat(void) {
    BoostConfig cfg = makeConfig();          // schedule null, vaneOpenCapPercent 55
    TEST_ASSERT_EQUAL_UINT8(55, vaneOpenCapForBoost(10.0f, cfg));
}

void test_vane_cap_exact_points(void) {
    BoostConfig cfg = makeConfig();
    cfg.vaneCapSchedule = kSchedule;
    cfg.vaneCapScheduleLen = 5;
    TEST_ASSERT_EQUAL_UINT8(22, vaneOpenCapForBoost(5.0f, cfg));
    TEST_ASSERT_EQUAL_UINT8(25, vaneOpenCapForBoost(10.0f, cfg));
    TEST_ASSERT_EQUAL_UINT8(29, vaneOpenCapForBoost(20.0f, cfg));
    TEST_ASSERT_EQUAL_UINT8(31, vaneOpenCapForBoost(25.0f, cfg));
}

void test_vane_cap_clamps_outside_ends(void) {
    BoostConfig cfg = makeConfig();
    cfg.vaneCapSchedule = kSchedule;
    cfg.vaneCapScheduleLen = 5;
    TEST_ASSERT_EQUAL_UINT8(22, vaneOpenCapForBoost(0.0f, cfg));   // below first
    TEST_ASSERT_EQUAL_UINT8(22, vaneOpenCapForBoost(2.0f, cfg));   // below first
    TEST_ASSERT_EQUAL_UINT8(31, vaneOpenCapForBoost(30.0f, cfg));  // above last
}

void test_vane_cap_interpolates_midpoint(void) {
    BoostConfig cfg = makeConfig();
    cfg.vaneCapSchedule = kSchedule;
    cfg.vaneCapScheduleLen = 5;
    // 12.5 psi: 25 + (2.5/5)*(27-25) = 26
    TEST_ASSERT_EQUAL_UINT8(26, vaneOpenCapForBoost(12.5f, cfg));
}
```
Add to `main()`:
```c
    RUN_TEST(test_vane_cap_null_schedule_falls_back_to_flat);
    RUN_TEST(test_vane_cap_exact_points);
    RUN_TEST(test_vane_cap_clamps_outside_ends);
    RUN_TEST(test_vane_cap_interpolates_midpoint);
```

- [ ] **Step 4: Run tests — verify they fail**

Run: `pio test -e native -f test_boost_bpr_logic`
Expected: FAIL — `undefined reference to vaneOpenCapForBoost` (link error).

- [ ] **Step 5: Implement the interpolator**

In `src/control/boostBprLogic.cpp`, add above `boostBprStep`:
```c
uint8_t vaneOpenCapForBoost(float boostPsi, const BoostConfig &cfg) {
    if (cfg.vaneCapSchedule == nullptr || cfg.vaneCapScheduleLen == 0) {
        return cfg.vaneOpenCapPercent;
    }
    const VaneCapPoint *pts = cfg.vaneCapSchedule;
    uint8_t n = cfg.vaneCapScheduleLen;
    if (boostPsi <= pts[0].boostPsi) return pts[0].capPercent;
    if (boostPsi >= pts[n - 1].boostPsi) return pts[n - 1].capPercent;
    for (uint8_t i = 1; i < n; i++) {
        if (boostPsi <= pts[i].boostPsi) {
            float pLow = pts[i - 1].boostPsi;
            float pHigh = pts[i].boostPsi;
            float cLow = (float)pts[i - 1].capPercent;
            float cHigh = (float)pts[i].capPercent;
            float cap = cLow + (boostPsi - pLow) / (pHigh - pLow) * (cHigh - cLow);
            return (uint8_t)(cap + 0.5f);
        }
    }
    return pts[n - 1].capPercent;  // unreachable; satisfies the compiler
}
```

- [ ] **Step 6: Run tests — verify they pass**

Run: `pio test -e native -f test_boost_bpr_logic`
Expected: PASS — all interpolator tests plus every pre-existing test.

- [ ] **Step 7: Commit**

```bash
git add src/control/boostBprLogic.h src/control/boostBprLogic.cpp test/test_boost_bpr_logic/test_boost_bpr_logic.cpp
git commit -m "feat(boost): pure vaneOpenCapForBoost interpolator + schedule config fields

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 2: Wire the schedule into `boostBprStep` + record `lastVaneCap`

**Files:**
- Modify: `src/control/boostBprLogic.cpp` (spool early-return records cap; final clamp uses the schedule cap)
- Test: `test/test_boost_bpr_logic/test_boost_bpr_logic.cpp` (schedule clamps the open-slam; `lastVaneCap` recorded)

**Interfaces:**
- Consumes: `vaneOpenCapForBoost(float, const BoostConfig&)`, `BoostState.lastVaneCap` (Task 1).

- [ ] **Step 1: Write the failing integration tests**

Append to the test file (and `RUN_TEST` in `main`):
```c
void test_schedule_clamps_open_slam_to_scheduled_cap(void) {
    BoostConfig cfg = makeConfig();
    cfg.vaneCapSchedule = kSchedule;
    cfg.vaneCapScheduleLen = 5;
    BoostInputs in;
    in.boostGaugePsi = 20.0f;   // schedule cap = 29
    in.tipGaugePsi = 30.0f;     // bpr 1.5 > target -> wants to open fully
    BoostState st = {0.0f, false, true, 0};
    uint8_t vane = boostBprStep(in, cfg, st, 0.01f);
    TEST_ASSERT_EQUAL_UINT8(29, vane);          // clamped to scheduled cap, not 55
    TEST_ASSERT_EQUAL_UINT8(29, st.lastVaneCap); // cap recorded for telemetry
}

void test_spool_region_records_spool_as_cap(void) {
    BoostConfig cfg = makeConfig();
    cfg.vaneCapSchedule = kSchedule;
    cfg.vaneCapScheduleLen = 5;
    BoostInputs in;
    in.boostGaugePsi = 1.0f;    // spool region
    in.tipGaugePsi = 0.5f;
    BoostState st = {0.0f, false, false, 0};
    uint8_t vane = boostBprStep(in, cfg, st, 0.01f);
    TEST_ASSERT_EQUAL_UINT8(cfg.spoolPercent, vane);
    TEST_ASSERT_EQUAL_UINT8(cfg.spoolPercent, st.lastVaneCap); // meaningful readout
}
```
Add to `main()`:
```c
    RUN_TEST(test_schedule_clamps_open_slam_to_scheduled_cap);
    RUN_TEST(test_spool_region_records_spool_as_cap);
```

- [ ] **Step 2: Run tests — verify the new ones fail**

Run: `pio test -e native -f test_boost_bpr_logic`
Expected: FAIL — `test_schedule_clamps_open_slam...` returns 55 (flat cap still used) and `lastVaneCap` is 0.

- [ ] **Step 3: Wire the schedule into `boostBprStep`**

In `src/control/boostBprLogic.cpp`, in the spool early-return block, record the cap before returning:
```c
    if (!state.inPiRegion) {
        state.wasSpooling = true;
        state.lastVaneCap = cfg.spoolPercent;   // meaningful readout in spool region
        return clampVane((float)cfg.spoolPercent,
                         cfg.vaneClosedPercent, cfg.vaneOpenPercent);
    }
```
Replace the final `return` (currently `return clampVane(vane, cfg.spoolPercent, cfg.vaneOpenCapPercent);`) with:
```c
    uint8_t cap = vaneOpenCapForBoost(in.boostGaugePsi, cfg);
    state.lastVaneCap = cap;
    return clampVane(vane, cfg.spoolPercent, cap);
```

- [ ] **Step 4: Run tests — verify all pass**

Run: `pio test -e native -f test_boost_bpr_logic`
Expected: PASS — all tests, including the pre-existing `test_bpr_above_target_opens` (schedule null → still clamps to `vaneOpenCapPercent`).

- [ ] **Step 5: Commit**

```bash
git add src/control/boostBprLogic.cpp test/test_boost_bpr_logic/test_boost_bpr_logic.cpp
git commit -m "feat(boost): clamp PI vane demand to boost-scheduled open cap

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 3: Production schedule table + `getActiveVaneCap()`

**Files:**
- Modify: `src/control/boostController.cpp` (define `vaneCapSchedule`, wire into `boostConfig`, add getter)
- Modify: `src/control/boostController.h` (declare getter)

**Interfaces:**
- Consumes: `VaneCapPoint`, `BoostConfig` schedule fields, `BoostState.lastVaneCap` (Tasks 1-2).
- Produces: `static uint8_t BoostController::getActiveVaneCap();`

- [ ] **Step 1: Declare the getter**

In `src/control/boostController.h`, add after `getIntegralTerm();`:
```c
        // Open-cap (%) the controller applied on the last update — for telemetry so
        // we can see the vane riding the boost-scheduled cap. See vaneOpenCapForBoost.
        static uint8_t getActiveVaneCap();
```

- [ ] **Step 2: Define the schedule table and wire it into the config**

In `src/control/boostController.cpp`, add above `static BoostConfig boostConfig`:
```c
// Boost (gauge psi) -> vane open-cap (%). THE tuning knob for the closure/slam
// strategy: edit these points + reflash. Ascending boost. Floor stays spoolPercent.
static const VaneCapPoint vaneCapSchedule[] = {
    { 5.0f, 22 }, { 10.0f, 25 }, { 15.0f, 27 }, { 20.0f, 29 }, { 25.0f, 31 },
};
static const uint8_t vaneCapScheduleLen =
    sizeof(vaneCapSchedule) / sizeof(vaneCapSchedule[0]);
```
Append the two new fields to the `boostConfig` aggregate initializer (after the `55` for `vaneOpenCapPercent`):
```c
    55,                  // vaneOpenCapPercent (flat fallback if schedule unset)
    vaneCapSchedule,     // vaneCapSchedule (boost-scheduled open cap; supersedes flat)
    vaneCapScheduleLen   // vaneCapScheduleLen
```
Update the `boostState` initializer to include the new trailing field:
```c
static BoostState boostState = {0.0f, true, false, 0};
```

- [ ] **Step 3: Implement the getter**

In `src/control/boostController.cpp`, add near the other accessors:
```c
uint8_t BoostController::getActiveVaneCap() { return boostState.lastVaneCap; }
```
In `BoostController::update()`, in the `#else` (MAP mode) branch, keep the readout sane by setting the cap to the flat fallback after the interpolate call:
```c
    appData.actuatorDemandedPosition = interpolate(boostGaugePsi);
    boostState.lastVaneCap = boostConfig.vaneOpenCapPercent;
```

- [ ] **Step 4: Build the firmware**

Run: `pio run -e teensy41`
Expected: SUCCESS (`[SUCCESS]`). No link/compile errors.

- [ ] **Step 5: Commit**

```bash
git add src/control/boostController.cpp src/control/boostController.h
git commit -m "feat(boost): production boost->open-cap schedule + getActiveVaneCap

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 4: Emit `vane_cap` in firmware telemetry

**Files:**
- Modify: `src/domain/ovgt.cpp` (add `Json_addUint("vane_cap", ...)` next to `pos_pct`)

**Interfaces:**
- Consumes: `BoostController::getActiveVaneCap()` (Task 3).

- [ ] **Step 1: Add the telemetry field**

In `src/domain/ovgt.cpp`, after the `act_temp` line, add:
```c
    Json_addUint("vane_cap", BoostController::getActiveVaneCap());  // active open-cap %
```
Confirm `boostController.h` is already included in this file (it is — `BoostController::getBprTarget()` etc. are used nearby).

- [ ] **Step 2: Build the firmware**

Run: `pio run -e teensy41`
Expected: SUCCESS. (NDJSON buffer is 1280 bytes; one small field is well within budget.)

- [ ] **Step 3: Commit**

```bash
git add src/domain/ovgt.cpp
git commit -m "feat(telemetry): emit vane_cap (active boost-scheduled open cap)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 5: Show `vane_cap` in the host telemetry tool

**Files:**
- Modify: `tools/ovgt-telemetry/src/types.ts` (add optional `vane_cap`)
- Modify: `tools/ovgt-telemetry/src/format.ts` (append cap to the actuator line)
- Test: `tools/ovgt-telemetry/src/format.test.ts` (assert cap renders + fallback)

**Interfaces:**
- Consumes: firmware `vane_cap` field (Task 4).

- [ ] **Step 1: Add the failing format assertions**

In `tools/ovgt-telemetry/src/format.test.ts`, in the first test (`formats core telemetry fields`), add:
```ts
  expect(lines).toContain("cap "); // vane_cap rendered on the actuator line
```
Add a new test:
```ts
test("vane_cap renders, and falls back to -- when absent", () => {
  expect(formatTelemetry({ ...sample, vane_cap: 29 }).join("\n")).toContain("cap 29");
  expect(formatTelemetry(sample).join("\n")).toContain("cap --");
});
```

- [ ] **Step 2: Run tests — verify failure**

Run (from `tools/ovgt-telemetry`): `npx vitest run src/format.test.ts`
Expected: FAIL — output has no `cap ` substring yet.

- [ ] **Step 3: Add the optional type field**

In `tools/ovgt-telemetry/src/types.ts`, after `act_temp?: number;`:
```ts
  vane_cap?: number; // active boost-scheduled vane open-cap %. Optional: absent in pre-2026-07-05 logs.
```

- [ ] **Step 4: Render it on the actuator line**

In `tools/ovgt-telemetry/src/format.ts`, change the BPR/actuator line to append the cap:
```ts
    `BPR ${s.bpr.toFixed(2)}/${s.bpr_target.toFixed(2)}   dem ${s.dem_pct}%  pos ${s.pos_pct}%  load ${s.act_load ?? "--"}  aTemp ${s.act_temp ?? "--"}  cap ${s.vane_cap ?? "--"}`,
```

- [ ] **Step 5: Run tests + typecheck — verify pass**

Run (from `tools/ovgt-telemetry`): `npx vitest run src/format.test.ts && npm run typecheck`
Expected: PASS, no type errors.

- [ ] **Step 6: Commit**

```bash
git add tools/ovgt-telemetry/src/types.ts tools/ovgt-telemetry/src/format.ts tools/ovgt-telemetry/src/format.test.ts
git commit -m "feat(telemetry): show vane_cap on host actuator line

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 6: Full verification + flash + on-truck sanity

**Files:** none (verification only)

- [ ] **Step 1: Full native test suite**

Run: `pio test -e native`
Expected: all suites PASS (no regression in boost/j1939/json/etc.).

- [ ] **Step 2: Full host test suite**

Run (from `tools/ovgt-telemetry`): `npm test`
Expected: all PASS (store/replay need local MongoDB at 127.0.0.1:27017).

- [ ] **Step 3: Firmware build (final)**

Run: `pio run -e teensy41`
Expected: SUCCESS.

- [ ] **Step 4: Flash (port must be free — stop telemetry TUI first)**

Free the port if held: `pkill -f "ovgt-telemetry.*index.tsx"` then flash:
Run: `pio run -e teensy41 -t upload`
Expected: `[SUCCESS]` and a `Programming...` trace (teensy-cli). If it reports success with no programming trace, VERIFY via Step 5 — do not assume.

- [ ] **Step 5: Verify the running firmware + new field**

Run: `stty -F /dev/ttyACM0 115200 raw -echo; timeout 4 cat /dev/ttyACM0 | head -c 600`
Expected: a `{"type":"t",...}` line containing `"vane_cap":22` (≈22 at idle) alongside `dem_pct`/`pos_pct`/`act_load`. Confirms the boost-scheduled cap is live.

- [ ] **Step 6: Restart the telemetry TUI (driver)**

The driver runs, in their own terminal: `cd tools/ovgt-telemetry && npm start`
Expected: actuator line now shows `... load <n>  aTemp <n>  cap <n>`. Watch `pos` riding toward `cap` under boost, plus TIT + `load` for EGT/strain.

- [ ] **Step 7: On-road tuning loop (driver)**

Drive and observe. To retune: edit the 5 points in `vaneCapSchedule` (`src/control/boostController.cpp`), rebuild + reflash (Steps 3-5). Revert entirely: `git checkout main` + reflash.
```

