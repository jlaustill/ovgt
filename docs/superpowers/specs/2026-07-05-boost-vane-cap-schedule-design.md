# Boost-scheduled vane open-cap — design

**Date:** 2026-07-05 · **Branch:** `feature/boost-vane-cap-schedule` · **Status:** approved, pre-implementation

## Problem

On the first loaded (trailer) run, the VGT actuator cannot hold the vanes at the
commanded closed angle against exhaust backpressure — `pos_pct` floats *open* of
`dem_pct`, scaling with boost (+10 at ~5 psi, +38 at 20+ psi; see memory
`project_actuator_closure_under_load`). A parked engine-idling bench self-test
confirmed the actuator closes fully with no backpressure (pos 1 @ demand 0), so
the fault is torque-vs-backpressure, not electrical/mechanical/decode.

The driving complaint is dynamic: the BPR PI loop **slams the vane open** (dumping
drive pressure), boost/BPR then collapses, and the loop **slams it back closed**
against pressure to rebuild — a violent open→re-close cycle that stresses the
actuator. Driving gently to avoid it is not acceptable (long trip ahead).

## Goal

Keep the vane on a **tight leash near closed** so the PI loop can no longer fling
it wide open, eliminating the open-slam and the subsequent close-under-pressure.
The leash **widens slightly as boost rises** (a little relief at high boost) and
must be trivially tunable in one place, and trivially revertible (feature branch).

## Approach

Replace the flat controller open-cap (`vaneOpenCapPercent = 55`) with a
**boost → open-cap schedule**, linearly interpolated. Only the *cap* (most-open
authority) changes; the *floor* stays `spoolPercent = 22`. Vane convention:
0% = fully closed (max drive/boost), 88% = fully open. Higher % = more open.

Resulting bands (floor 22 → scheduled cap):

| Boost (psi) | open cap % | vane band |
|---|---|---|
| ≤ 5  | 22 | pinned 22 |
| 10   | 25 | 22–25 |
| 15   | 27 | 22–27 |
| 20   | 29 | 22–29 |
| ≥ 25 | 31 | 22–31 |

### Chosen implementation: nullable schedule on `BoostConfig` (backward compatible)

Add the schedule as **trailing, nullable fields** on `BoostConfig` rather than
removing `vaneOpenCapPercent`. When the schedule pointer is null (every existing
test), behavior is identical to today's flat cap — so the validated `boostBprStep`
test suite is untouched. Considered but rejected: (B) replacing the scalar
outright (breaks all test initializers for no benefit); (C) hardcoding the table
inside `boostBprLogic.cpp` (not per-config, not native-testable in isolation, and
the "one place to tweak" ends up buried in control logic).

## Components

### 1. Schedule constant — `src/control/boostController.cpp`
The single tweakable table, editable + reflash (mirrors the legacy `pressureMap`
pattern):
```c
static const VaneCapPoint vaneCapSchedule[] = {
    { 5.0f, 22 }, { 10.0f, 25 }, { 15.0f, 27 }, { 20.0f, 29 }, { 25.0f, 31 },
};
```
Wired into `boostConfig` (schedule pointer + length).

### 2. Pure interpolator — `src/control/boostBprLogic.{h,cpp}` (native-tested)
```c
struct VaneCapPoint { float boostPsi; uint8_t capPercent; };

// Boost -> open-cap %, linear between ascending points; flat outside the ends.
// Falls back to cfg.vaneOpenCapPercent when no schedule is wired.
uint8_t vaneOpenCapForBoost(float boostPsi, const BoostConfig &cfg);
```
- `boostPsi ≤ points[0]` → `points[0].capPercent`.
- `boostPsi ≥ points[last]` → `points[last].capPercent`.
- otherwise linear interpolation between bracketing points, rounded to nearest.
- schedule null / len 0 → `cfg.vaneOpenCapPercent` (existing behavior).

`BoostConfig` gains: `const VaneCapPoint *vaneCapSchedule;` and
`uint8_t vaneCapScheduleLen;` (trailing; default null/0).

### 3. Wire-in — `boostBprStep`, final clamp (currently line 80)
```c
uint8_t cap = vaneOpenCapForBoost(in.boostGaugePsi, cfg);
return clampVane(vane, cfg.spoolPercent, cap);
```
No other line changes. Spool region (boost < ~3 psi) still returns `spoolPercent`
before the cap is consulted; the spool-protect clamp (<6 psi) already pins to 22 —
consistent with the schedule reading 22 at 5 psi.

### 4. Telemetry — `vane_cap` (observability for tuning-as-you-drive)
Firmware emits the active cap in the 10 Hz line; host shows it on the actuator
display line so `dem` riding the cap is visible against boost.
- `src/domain/ovgt.cpp`: `Json_addUint("vane_cap", BoostController::getActiveVaneCap())`.
  `boostBprStep` records the cap it used into `BoostState` (new
  `uint8_t lastVaneCap` field); `boostController.cpp` exposes it via
  `BoostController::getActiveVaneCap()`. In the spool region the recorded cap is
  the spool position's effective ceiling (`spoolPercent`), so the readout is
  always meaningful.
- `tools/ovgt-telemetry/src/types.ts`: `vane_cap?: number` (optional; absent in
  older logs).
- `tools/ovgt-telemetry/src/format.ts`: append `cap ${s.vane_cap ?? "--"}` to the
  actuator line.

## Safety

Per decision: **no automatic EGT escape** this iteration. The schedule holds the
vane more closed under load than today's flat cap, biasing toward higher EGT
(today's drive touched TIT 887 °C; 17.6 % > 760 °C). Mitigation is the driver
watching live TIT + `act_load` and reverting the branch if EGT misbehaves.
Documented explicitly so a future iteration can add an EGT-relief override or a
boost/BR ceiling (the design doc's current stance is deliberately "BPR only, no
boost ceiling").

## Testing (native, `pio test -e native -f test_boostBpr...`)

New unit tests, TDD:
1. `vaneOpenCapForBoost`: exact points (5→22 … 25→31); midpoints (7.5→~23,
   12.5→26); below-min (0→22) and above-max (30→31); null-schedule fallback →
   `vaneOpenCapPercent`.
2. `boostBprStep` with schedule wired: at a boost where cap < 55, a BPR error that
   would drive the vane open past the cap is clamped to the scheduled cap (not 55);
   the floor still holds at `spoolPercent`.
3. Existing `boostBprStep` tests (schedule null) remain green, proving backward
   compatibility.

## Rollout / revert

- Branch `feature/boost-vane-cap-schedule` off `main`.
- Flash: `pio run -e teensy41 -t upload`. Restart telemetry TUI to see `vane_cap`.
- Revert: `git checkout main` + reflash. No data-model migration; `vane_cap` is
  additive/optional.

## Out of scope (YAGNI)

- EGT-relief override / boost ceiling (noted for a future iteration).
- Making the floor boost-dependent (floor stays 22 — validated; 18 spiked EGT).
- Any runtime/serial tuning (compile-time only — driving-hazard policy).
