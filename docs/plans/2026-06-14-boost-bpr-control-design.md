# Boost Control via BPR=1.0 — Design

## Summary

Replace the open-loop boost→vane lookup map with a closed-loop controller that
targets a **backpressure ratio (BPR) of 1.0** — i.e. holds drive pressure equal
to boost. BPR is a universal, physics-based setpoint, so it doesn't need the
per-condition re-tuning the lookup map requires (the reason for the change).

Below a boost threshold (where BPR is unreliable) the controller holds a fixed
spool position; above it, a PI loop modulates the vanes to keep BPR at 1.0. The
PI mirrors the exhaust brake's structure (`exhaustBrakeLogic`) but is its own
module — the brake is mid-tuning on the truck, so we don't refactor it yet.

## Definitions

- **BPR** = `turbineInputPressure(gauge) / boost(gauge)` = drive pressure ÷ boost
  (same as the existing `BPR:` debug field). Boost gauge = `COP − CIP`.
- **Direction:** closing vanes raises drive pressure faster than boost → BPR up.
  So `BPR < 1 → close vanes` (raise drive), `BPR > 1 → open vanes`. This is the
  same error→closure sense as the brake.

## Decisions (from brainstorming)

- **Why switch:** the lookup map needs constant manual re-tuning (fuel, altitude,
  mods); a BPR=1.0 target is universal.
- **No boost cap (deliberate):** the PI targets BPR=1.0 with no over-boost ceiling.
  Chasing BPR by closing vanes also raises boost, so this *can* over-boost at high
  load. Omitted intentionally (monitored test truck); a boost-ceiling override
  (open vanes if boost > max) is the obvious future guard if a pull runs away.
- **Low-boost fallback:** fixed spool position below the threshold, BPR PI above,
  with a bumpless handoff.

## Architecture

Mirrors the brake's pure-logic + glue split.

### `boostBprLogic` (new, `src/control/boostBprLogic.h/.cpp`, pure C++, native-tested)

```cpp
struct BoostInputs {
    float boostGaugePsi;   // (COP - CIP) in psi
    float tipGaugePsi;     // turbine inlet (drive) pressure, gauge psi
};

struct BoostConfig {
    float   bprTarget;       // 1.0
    float   boostMinPsi;     // below this -> spool region (BPR unreliable)
    uint8_t spoolPercent;    // fixed vane position while spooling
    uint8_t vaneClosedPercent; // from VaneConfig.h
    uint8_t vaneOpenPercent;   // from VaneConfig.h
    float   kp;              // ~88 scale (BPR error is a 0..1 ratio)
    float   ki;
};

struct BoostState {
    float integralTerm;   // closure contribution, vane-% units
    bool  wasSpooling;    // for bumpless spool->PI handoff
};

// Returns the demanded vane position. Caller (BoostController) only invokes this
// when NOT in manual mode and NOT braking, so no mode/active flag is needed.
uint8_t boostBprStep(const BoostInputs &in, const BoostConfig &cfg,
                     BoostState &state, float dtSeconds);
```

**Logic:**
1. **Spool region** (`boostGaugePsi < boostMinPsi`): return `spoolPercent`; set
   `wasSpooling = true`. (BPR is garbage at low boost — don't compute it.)
2. **BPR region** (`boostGaugePsi >= boostMinPsi`):
   - **Bumpless handoff:** if `wasSpooling`, seed `integralTerm = vaneOpenPercent
     − spoolPercent` and clear `wasSpooling`, so the PI's first output starts at
     `spoolPercent` instead of jumping to fully open. (Same lesson as the brake's
     engagement jump.)
   - `bpr = tipGaugePsi / boostGaugePsi` (safe — boost ≥ threshold > 0)
   - `error = bprTarget − bpr`
   - `integralTerm = clamp(integralTerm + ki·error·dt, 0, travel)` where
     `travel = vaneOpenPercent − vaneClosedPercent` (anti-windup)
   - `closure = kp·error + integralTerm`
   - `return clamp(vaneOpenPercent − closure, vaneClosedPercent, vaneOpenPercent)`

   `BPR < 1 → error > 0 → more closure → lower (closed) vane → drive pressure up
   → BPR toward 1`. No derivative term (noisy ratio).

### `BoostController` glue (`src/control/boostController.cpp`, existing — rewritten)

Keeps the `Initialize()/update()` interface so `ovgt.cpp` arbitration is
unchanged (`if (!manualMode && !braking) BoostController::update();`). `update()`:
- reads `appData`: `boostGaugePsi = (COP − CIP) · HPA_TO_PSI`,
  `tipGaugePsi = turbineInputPressureHpa · HPA_TO_PSI`,
- computes `dt` from `millis()`,
- calls `boostBprStep`, writes `appData.actuatorDemandedPosition`.

The `PressureMapEntry` lookup map and `interpolate()` are removed.

### Config (`BoostConfig` defaults; tunable on the truck)

| Field | Default | Notes |
|-------|---------|-------|
| `bprTarget` | 1.0 | the whole premise |
| `boostMinPsi` | 2.0 | spool/BPR boundary — tune |
| `spoolPercent` | 25 | current idle/spool position — tune |
| `vaneClosedPercent` / `vaneOpenPercent` | from `VaneConfig.h` | 0 / 88 |
| `kp` | ~88 | maps a full 1.0 BPR error to full vane travel — tune |
| `ki` | small (e.g. 20) | trims steady-state — tune |

**Gain scale note:** BPR error is a ratio (~0–1), not psi. So `kp` is ~88 (≈ full
travel per unit BPR error), *not* ~2 like the brake. Same structure, different
scale — which is why the PI is mirrored, not shared (yet).

## Testing

Native Unity tests for `boostBprLogic` (pure, no hardware):
- Spool region: `boost < boostMinPsi` → returns `spoolPercent`.
- Direction: `BPR < 1` (e.g. `tip = 0.5·boost`) → vane closes (< open);
  `BPR > 1` → vane opens (toward open).
- Bumpless handoff: first BPR cycle right after spool → vane ≈ `spoolPercent`
  (not a jump to fully open).
- Anti-windup + output clamp to `[closed, open]`.

## Integration & Out of Scope

- Arbitration in `ovgt.cpp` is unchanged; this only swaps what `BoostController`
  computes. Exhaust brake is untouched.
- **Out of scope:** the boost-ceiling over-boost guard (deliberately omitted);
  extracting a shared `piStep` primitive used by both brake and boost (do once
  both loops are tuned); converting boost to C-Next (later migration step).
- **Parallel (not this spec):** the Teensy thermal issue — relocate the
  controller to a cooler spot (under the fender) and revert to 150 MHz; the engine
  bay over-heats even a healthy chip under sustained load.

## Addendum (2026-06-14): compile-time hedge + live tuning

Two changes to the above, to de-risk on-truck validation:

- **Compile-time mode switch** `BOOST_USE_BPR` in `boostController.cpp`: 1 = the
  BPR PI loop (this design); 0 = the legacy boost→vane lookup map, kept as a
  fallback. The unused path is `#if`-compiled out (no dead code). Flip + reflash to
  switch — the map is **not** removed, it is the hedge.
- **Runtime tuning over serial** (BPR builds): `bpr <v>`, `kp <v>`, `ki <v>` mutate
  the live `BoostConfig` so the target and gains can be swept without reflashing;
  `params` prints the current mode/values; the 1 Hz debug line shows
  `BPR:<measured>/<target>`. `spoolPercent` and the thresholds stay compile-time.

Note: the **150 MHz thermal revert** mentioned above was dropped — the overheat was
a placement issue (proximity to the turbo during sustained high-EGT grade climbs),
not the clock; the fix is relocating the controller. Clock stays at 600 MHz.

## Addendum 2 (2026-06-15): on-truck validation + robustness pass

Validated under sustained load (65 mph cruise): the loop **locks BPR = 1.00** with
the vane parked at ~33% across a 10–15 psi boost range (TIP == boost). The concept
and gains are proven. Three refinements from the road data, all in `boostBprLogic`:

- **Gains baked**: `kp` 88 → **20**, `ki` **20**. kp=88 was effectively bang-bang
  (a 1.0 BPR error saturated the whole travel); kp=20 glides. These are now the
  compiled defaults so reboots come up dialed (still live-tunable).
- **Hysteresis on the spool/PI boundary**: a single threshold made the vane chatter
  spool↔open when boost danced across it. Replaced `boostMinPsi` with two
  thresholds — engage PI above `boostPiPsi` (3.0), fall back to spool below
  `boostSpoolPsi` (1.5), hold the region in the dead band.
- **Actuator-tracking anti-windup**: at light cruise (BPR=1.0 unreachable) the
  integral wound the vane shut while the actuator lagged, then dumped a
  drive-pressure spike (BPR→2.0) on the next tip-in. The integral now only
  accumulates while the actuator has reached the last command (within
  `integralTrackingBand`), so it can't wind against a slewing/stuck actuator. Needs
  `actuatorReportedPercent` (from `appData.actuatorReportedPosition`) as an input.

Thermal: relocating the controller forward by the airbox (~1 ft off the turbine)
held the MCU at 42–51 °C through 740 °C-EGT pulls — placement confirmed as the fix.
