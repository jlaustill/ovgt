# EPIC — Real-Time Compressor Efficiency (CE) for Vane Control

**Status:** Proposed (epic / umbrella). Spawns per-phase design+plan pairs.
**Created:** 2026-06-22 · **Revised:** 2026-06-23 (multi-agent research + adversarial review folded in)
**North star:** `docs/north-star.md` — CE is the project's objective function; this epic is the path to making it dependable enough to control on.
**Builds on:** `docs/plans/2026-06-16-ce-accuracy-design.md` / `-plan.md` (the cotSettle trust-flag + τ measurement) — the deferred "follow-on for live/transient CE."

---

## Vision

Today CE is trustworthy only at rare **settle points** (3 in a 23-minute drive — and
the settle flag itself is buggy, below). The goal is **CE accurate enough to close a
vane-control loop on** — the prerequisite for the north star (maximize CE at all
times). Get there by (Phase 0) measuring each sensor's time behavior dependably,
(Phase 1) time-aligning the inputs in software, (Phase 2) shrinking COT's lag in
hardware, and (Phase 3) closing the control loop.

CE is **telemetry/display only** (`src/domain/ovgt.cpp:74-75`); no control loop reads
it. So Phases 0–2 carry **zero control risk** — but **not zero effort/sequencing
risk**: see the hard gate below.

> **⚠ The "offline-first" gate (corrected).** Existing logs **cannot** be used to fit
> alignment: COP/CIP/CIT are already EMA-filtered *and* the whole stream is decimated
> to 10 Hz (`src/domain/ovgt.cpp:44`, `count%10`). 10 Hz cannot resolve 21 °C/s slew
> or sub-100 ms group delay (Nyquist). **Firmware re-instrumentation (despike-only
> filtering + higher-rate/burst logging) + a fresh drive is a hard prerequisite before
> any offline alignment work.** "Offline-first, zero risk" is true for *control* risk,
> not for sequencing.

---

## The core insight (corrected & quantified)

```
CE = idealRise / actualRise = [ CIT_K·(BR^0.286 − 1) ] / ( COT_K − CIT_K ),  BR = COP/CIP
```

CE is a **single-instant ratio** of four inputs; time-skew between them produces
garbage (CE > 100 % in transients — which the firmware emits *uncapped*, a free
skew/heat-soak detector). Verified facts that shape the plan:

| Fact (verified) | Consequence |
|---|---|
| Sensor noise after light filtering ⇒ **~0.1–0.4 % CE RMS**; COT lag ⇒ **+5–28 % CE** in tip-ins | Lag dominates noise ~100×. Objective = **match group delay, then despike-only**, not a symmetric "balance." |
| Transient error is the **COP−COT group-delay *difference***, not COT's absolute lag (matched ⇒ ~2 % error; 1 s mismatch ⇒ ~274 %) | Headline metric for Phase 1 is *delta* delay between the fast movers. |
| **COP and CIP share the same EMA** and have equal-and-opposite CE elasticity | Their lags largely **cancel in BR** — BR is already roughly self-consistent (free simplification). |
| The denominator is **COT − CIT**, two *different* sensor types (K-type vs NTC) | The **temperature pair's** mutual alignment matters as much as COP-vs-COT. |
| **CIP/CIT are co-equal for ACCURACY** (CIT per-°C leverage = `1+rise/Tin` > COT at *every* op point; CIP elasticity ~2.9 at cruise) … | …so they are **not** "free" — they must be carried to a common timestamp and calibrated well. |
| …but **second-order for TIMING** (real log: CIP moved ~0.7 psi all drive, never sub-ambient) | Aligning the **two fast movers first is correct**; CIP/CIT get a small fixed lag term. |
| **Software lag is removable only for the PRESSURE channels** (intrinsic τ <10 ms); COT/CIT thermal mass is irreducible in software | EMA removal fixes BR; the **temperature lag** (which gates `actualRise`) needs the *hardware* lever. |
| Pressure→temperature is an instantaneous **nonlinear (BR^0.286)** coupling, not a time shift | COP↔COT cross-correlation is a **heuristic**, not the τ truth. Bench step + reference probe is the dependable primary. |
| **MAX31856 ≈ 10–12 Hz / ~100 ms floor** (one-shot is *slower*); telemetry 10 Hz | Alignment can't be finer than ~100 ms. A faster COT probe needs a **faster ADC front-end**, not a config tweak. |

**Package deal (unchanged, reaffirmed):** strip COP's sensor-layer EMA and add
CE-stage alignment **together** — stripping alone *increases* skew (the accidental
EMA match is part of why CE works at all today).

---

## Phase 0 — Foundational sensor time-drift measurement (the prerequisite)

> The user's repeatedly stated requirement: a **rock-solid, dependable, accurate** way
> to measure each sensor's response time / time-drift before any alignment.

- **0.1 — Fix the trust gate first (offline, no firmware, do now).** The settle flag
  never clears (0/854). `cotSettleStep` is pure in `(cotC, boostPsi, dt)` and the
  existing 10 Hz log carries `cot_c/cop_hpa/cip_hpa/t_ms` — **re-run the state machine
  in the host tool to root-cause it with zero firmware change.** Until fixed,
  settle-point CE is not usable as ground truth for anything downstream.
- **0.2 — Measure the EMA group delay (on-device, cheap).** The α=0.01 EMA delay is
  unknown (0.46 s @215 Hz/ch … ~1.9 s @53 Hz/ch) and gates COT's match target.
  Realized per-channel rate is likely well below 215 Hz (single-shot + default 100 kHz
  I²C, no `Wire.setClock`; `CONVERSION_TIMEOUT_MS=5` drops samples). Measure it with
  the existing DWT cycle counters (count completed conversions/sec), not a step-fit.
- **0.3 — Re-instrument firmware for measurability (the hard gate).** Despike-only
  sensor filtering (so the log carries wide `dCOP/dt`) **and** a higher emit rate or a
  bounded high-rate **burst mode** for COP step/alignment windows. Guard burst mode:
  manual mode only, bounded duration, verify 100 Hz loop-period stability (USB-CDC
  backpressure can stall the actuator/boost/CAN loop and jitter the very timestamps
  being measured).
- **0.4 — Bench step-response = the dependable absolute-τ primary.** Per sensor
  *type*, with a fast reference and a GPIO t=0 marker: pressure via solenoid/ball-valve
  step from a tank (isolates the pressure-channel software lag, sensor τ <10 ms);
  thermal via mechanized plunge (boiling↔ice) **and** an air-to-air heat-gun step (do
  both: liquid = tight lower-bound CI, air = representative). 10–20 reps → `τ ± 95 % CI`.
- **0.5 — In-vehicle relative lag via scripted vane-snap.** Command a deterministic
  vane-snap pattern (manual mode + `actuatorDemandedPosition` already exist) for
  repeatable boost steps with a logged t=0. Estimate the **relative-lag matrix** (COT,
  CIP, CIT each vs the COP reference) — treating cross-correlation/63.2%-crossing as a
  *heuristic* anchored by the bench τ, not as the truth (pressure→temp warp confound).
- **Done when:** per-sensor `τ ± CI`, the realized EMA delay, and the COP-referenced
  relative-lag matrix — each with an uncertainty — plus a fixed trust gate.

**Open decision that scopes everything below:** align out **sensor lag only**
(delay-shift) or **sensor + thermodynamic physics** (needs a physics model)? CE
consumes both; the answer picks the methodology.

## Phase 1 — Software time-alignment → *semi-accurate* transient CE

- **1.1 — (gated on 0.3)** Confirm the re-instrumented log carries raw/wide-bandwidth
  COP. Note: a "raw COP" field at 10 Hz is still ~21× under the ADS native rate —
  **relaxing the EMA is the substantive lever**, not just adding a field.
- **1.2 — Offline CE recompute + alignment in `tools/ovgt-telemetry`** (host does no CE
  math today). Match COP's group delay to COT's effective τ (recommended: matched
  first-order low-pass; *rejected:* COT deconvolution — noise blow-up; pure delay — COT
  isn't a pure delay). Optionally regime-adaptive (heavy filter when flat, light when
  slewing) using cotSettle's slope detection.
- **1.3 — Validate** against trusted ground truth. ⚠ Settle points **exclude**
  transients and are currently unreliable (0.1). Use the **bench step with a reference
  probe** (the 2.1 co-located test doubles as this) and settle-point-bracketed ramps as
  the transient truth source — this is a gating dependency, not an afterthought.
- **1.4 — Add CE diagnostics** the physics gives for free: surface **CE > 1.0** as a
  skew/non-adiabatic flag; add a **heat-soak** magnitude correction or **CIT-vs-ambient
  trust gate** (heat-soak biases CE low independent of timing); decide whether `ce_pct`
  should be gated/blanked when `!ce_settled` or stay advisory.
- **1.5 — Resolve the CIT dual-writer** before trusting CIT: the live path is the ADS
  NTC (`adcSensors.cpp:186`, float); `citSensor.cpp` (MAX31856, int16 cast) is a dead
  duplicate commented out at `src/domain/ovgt.cpp:113,206`. One writer of one field.
- **1.6 — Port validated alignment to firmware** (C-Next), emitting an **aligned CE**
  alongside the gated one. Still display/telemetry — no control risk.

## Phase 2 — Hardware/sensor iteration → *approach real-time* CE

- **2.1 — Faster COT probe + co-located timing test.** Two **no-new-ADC** paths:
  exposed-junction K-type (drop-in on the existing MAX31856) **or** an NTC on **ADS2
  (0x49) ch3** (currently a temporary tenant for lift-pump; that channel is earmarked
  for COT — corrects the old design's "needs a 3rd ADS"). Co-locate the candidate with
  the production K-type at the outlet; the τ **difference** common-mode-rejects gas
  dynamics → the true added probe lag. (This is also the NTC-vs-thermocouple timing
  test CIT can never provide.)
- **2.2 — Mind the ADC floor.** The MAX31856's ~100 ms cadence becomes the limiter once
  probe τ drops below ~0.5 s (one-shot is slower, not faster — no config escape). The
  **NTC-on-ADS path round-robins faster** and is the way to actually exploit a fast
  probe; a sub-100 ms goal may need a dedicated fast ADC. Fix upstream, never hack.
- **2.3 — Absolute calibration** (distinct from response time): a 5 °C COT bias at a
  60 °C rise ≈ 8 % CE error. Calibrate against a reference in the operating range;
  for the NTC, calibrate at in-circuit divider current (self-heating).
- **2.4 — Re-tune the alignment** for the faster probe and quantify residual lag /
  transient CE error per candidate.

## Phase 3 — CE-based vane control (the north-star payoff)

- **3.1 — Objective & constraints.** Constrained CE-maximization subject to surge
  margin, boost/torque demand, EGT/backpressure, transient spool; decide CE vs engine
  BTE as the true objective; define a **fallback mode** for the BR≤1 region where CE is
  undefined (idle/cruise/overrun).
- **3.2 — Missing measurement: airflow.** The compressor-map x-axis and any surge
  margin need corrected mass flow, which the truck lacks. Reconstruct an **empirical**
  efficiency map from logged data + a **speed-density airflow estimate** (J1939 RPM +
  MAP + IAT + VE map) or a J1939 MAF. Detect **surge by its low-frequency boost
  oscillation on the fast COP channel**, not a flow margin.
- **3.3 — Control architecture.** **Map-based feedforward + PI, gain-scheduled by
  RPM/load, as the production PRIMARY** (a road truck rarely holds an operating point
  long enough for extremum-seeking; COT caps the bandwidth). **Extremum-seeking control
  (ESC) as a secondary** quasi-steady/dyno refinement. **MPC out of scope** for Teensy
  4.1. Do not close the loop until aligned CE meets the acceptance bar.
- **Acceptance bar (north-star "done"):** CE phase lag well inside the loop time
  constant and CE noise below ~½ the control perturbation, during real transients.

---

## Sequencing & dependencies

```
0.1 fix trust gate (offline, now) ─┐
0.2 measure EMA delay ─────────────┤
0.3 re-instrument fw (HARD GATE) ──┼─► 0.4 bench τ ─┬─► 0.5 in-vehicle relative-lag matrix
                                   │                │
                                   └────────────────┴─► 1.x software align (offline → firmware)
                                                              │
2.1 faster probe + co-located τ ──────────────────────────────┼─► 2.4 re-tune ─► 3.x control
        (also serves as 1.3 transient ground truth)            │
```
Phase 1 delivers value on current hardware and is independent of the RIFE probe. Phase
2 reuses Phase 1's alignment machinery with a smaller COT τ. Phase 3 is gated on
aligned CE meeting the acceptance bar **and** an airflow estimate.

## Decisions to confirm (need user input)
1. **Alignment target:** sensor-lag-only (delay-shift) vs sensor+physics (model)? Scopes Phase 0/1 methodology.
2. **True objective:** compressor CE, or engine brake-thermal-efficiency (CE is its dominant lever but the optima differ slightly)?
3. **"Semi-accurate" tolerance** for Phase 1 (aligned transient CE within X % of trusted CE) and the CE-noise ceiling for despike-only.
4. **Faster-COT path:** exposed K-type (drop-in, MAX31856-floored) vs NTC-on-ADS (faster cadence, needs lift-pump to vacate ch3)?
5. **`ce_pct` gating:** blank when `!settled` in firmware, or keep advisory and filter host-side?

## Risks
- COT not cleanly first-order → a single matched-τ low-pass under/over-corrects (consider 2-pole). Distinguish sensor τ from gas dynamics (cooldowns measure gas, not the probe).
- **Ground-truth scarcity** for transient validation (settle points exclude transients and are currently unreliable) → commit the bench-reference-probe truth source.
- **No airflow** → surge margin and map placement not computable without a speed-density/MAF estimate.
- Heat-soak / low-flow makes CE meaningless regardless of timing — a hard constraint on "CE at all times" (needs the fallback mode + heat-soak gate).
- Burst-logging USB-CDC backpressure stalling the control loop — gate it.

## Definition of done (epic)
A vane-control loop that holds the compressor near its efficiency optimum across the
drivable envelope (with a defined fallback where CE is undefined), driven by an
aligned, calibrated, real-time-enough CE whose residual error and lag meet the
acceptance bar — validated offline-then-firmware, with the probe-timing and airflow
prerequisites in place.
