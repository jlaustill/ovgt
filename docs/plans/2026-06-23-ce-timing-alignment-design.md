# CE Timing-Alignment — Phase 0/1 Design

**Status:** Design (precedes a task-by-task plan). Adversarially reviewed 2026-06-23 (math / codebase-feasibility / completeness); corrections folded in.
**North star:** `docs/north-star.md` · **Epic:** `docs/plans/2026-06-22-realtime-ce-accuracy-epic.md` (concrete design for Phase 0 "measure sensor time-drift" + Phase 1 "software alignment").

> **Naming:** a sensor's lag / time constant is called **`responseTime`** throughout
> (seconds) — spelled out, never `tau`/`τ`.

## Goal

Compute a **time-aligned CE** by measuring each sensor's response time and delaying the
faster channels so all four inputs describe the *same physical instant*:

```
delay_i      = responseTimeMax − responseTime_i        (responseTimeMax = max over the 4 sensors)
aligned CE   = f( COT(now),  COP(now − delay_COP),  CIP(now − delay_CIP),  CIT(now − delay_CIT) )
```

**Derivation (verified):** COT(now) reflects reality at `now − responseTime_COT`; we
want every other channel's value reflecting that *same* instant, so read channel *i* at
time `t` where `t − responseTime_i = now − responseTime_COT` →
`t = now − (responseTime_COT − responseTime_i)`. With COT slowest,
`responseTime_COT = responseTimeMax`, giving `delay_i = responseTimeMax − responseTime_i ≥ 0`
(so we only ever look *backward* — never extrapolate). The aligned CE is **correct but
delayed by responseTimeMax**; shrinking that delay is the hardware lever (epic Phase 2),
not this design.

**Key quantity:** the per-sensor **lag ≈ first-order response time** (the 63.2 % step
time), *not* "settle-to-flat" (≈3–5× the response time). Group delay of
`1/(1 + responseTime·s)` is `responseTime/(1 + (ω·responseTime)²)` → **responseTime as
ω→0** — so delay-to-slowest aligns the **low-frequency** component exactly; genuinely
fast transients keep a residual high-frequency skew (see *Known error floors*).

CE is display-only, so all of this is **built and validated offline first** in
`tools/ovgt-telemetry`, then ported to firmware — zero control risk.

**Time base:** everything is in **seconds**. Telemetry `t_ms` (ms) → seconds via
`/1000`. (The firmware `cotSettle` already works in seconds.)

---

## Part 1 — The response-time calculator (sensor-agnostic)

A pure, vitest-tested host module (`tools/ovgt-telemetry/src/responseTime.ts`) that fits
a first-order response time to one clean step response, plus an aggregator for repeated
steps. It generalizes the firmware's 63.2 %-crossing extractor (`cotSettle.cpp:70-89`,
`extractTau`) into a full least-squares fit with a goodness-of-fit so bad steps are
*rejected*, not silently trusted.

```ts
export interface StepSample { t: number; v: number; }   // t SECONDS; v in ENGINEERING
                                                         // units (°C or hPa — see below)
export interface ResponseTimeFit {
  responseTime: number;   // first-order time constant (s)
  step: number;           // signed step size (v units)
  t0: number; vFinal: number;
  rmse: number;           // fit residual (v units)
  r2: number;             // goodness of fit
}
export interface ResponseTimeResult { responseTime: number; ci95: number; n: number; rejected: number; }

// Fit T(t)=vFinal+(v0−vFinal)·exp(−(t−t0)/responseTime). Seed from the 63.2 % crossing,
// refine with golden-section/Gauss–Newton minimizing RMSE.
// RETURNS null (rejects) when: r2 < ~0.95 (not first-order), |step| < minStep, OR the
// 63.2 % target is never reached within the captured window (truncated step). Do NOT
// inherit the firmware fallback (response time = measElapsed, cotSettle.cpp:73), which
// under-reports.
export function fitFirstOrderResponseTime(samples: StepSample[], opts?: { minStep?: number }): ResponseTimeFit | null;

export function aggregateResponseTime(fits: ResponseTimeFit[]): ResponseTimeResult;  // mean ± 95 % CI, drops nulls
```

**Fit in engineering units, not volts.** `cit_c`/`cot_c` are °C and `cip_hpa`/`cop_hpa`
are hPa — a first-order *bead/element* is first-order in those units. (Fitting raw NTC
volts would be non-exponential because of the divider + Steinhart-Hart nonlinearity, and
the fit would wrongly reject it.) The host reads each channel from the NDJSON; per-sample
`dt` comes from successive `t_ms` deltas (the firmware's exact `cotSampleDt` is not
emitted — fine for offline fitting).

A capture-side helper slices step windows out of a logged drive/bench file:

```ts
// Channel names are the ACTUAL telemetry fields (types.ts:5-7): cit_c, cot_c (°C),
// cip_hpa, cop_hpa (uint16 ABSOLUTE hPa). There is NO cop_psi/cip_psi field.
type Channel = "cit_c" | "cip_hpa" | "cot_c" | "cop_hpa";
// Bench: split on the GPIO t=0 marker records (Part 2). In-vehicle COT: arm on the
// boost-flat edge, mirroring cotSettle — using the RAW adjacent-sample boost slope
// (boost_slope_psi_s as emitted, ovgt.cpp:202), NOT the windowed slope the settled flag
// uses, so the host reproduces the firmware's arming exactly.
export function extractSteps(capture: ParsedLine[], mode: "marker" | "boostFlat",
                             channel: Channel): StepSample[][];
```

**Tests:** synthetic first-order step → response time within tolerance; noisy step →
within CI; two-pole/ramp input → low r2 → rejected; truncated step → null (not seed
fallback); `aggregateResponseTime` CI shrinks with N.

---

## Part 2 — Acquiring the response time for each sensor

| Sensor | How | Why |
|--------|-----|-----|
| **COT** | **In-vehicle** (preferred) — boost-flat reference on real transients (what `cotSettle` does); bench fallback | Only channel that fast-steps in-vehicle; measured at *representative gas flow* (response time is flow-dependent — below) |
| **COP** | **Bench** pressure step — **burst-logged**, see warning | Fastest signal; at 10 Hz a real COP step looks instantaneous |
| **CIP** | **Bench** pressure step | Never steps in-vehicle (moved 0.7 psi all of drive-2) |
| **CIT** | **Bench** thermal step (fit in °C) | Never steps in-vehicle (≤0.53 °C/s) |

**⚠ Measure on the signal path you will ship — and burst-log the pressure step.** Today
COP/CIP/CIT carry the shared voltage-level EMA (`adcSensors.cpp:125`, α=0.01 = ~100 *ADC
conversions* to 63 %; its wall-clock response time depends on the round-robin per-channel
rate, NOT 10 Hz — so the often-quoted "~0.5 s" is rate-dependent, measure it). The plan is
to relax this EMA to despike-only (epic Story 1.1). **After de-EMA the pressure element's
intrinsic response time is <10 ms — which 10 Hz telemetry CANNOT resolve.** Since
**COP→COT is the one pair that actually moves CE**, do **not** just "assume responseTime_COP
≈ 0": either **burst-log** the bench pressure step (~215 Hz, epic Story 0.3) to measure the
de-EMA'd value, or explicitly bound it and carry that as the alignment error. `delay_COP`
wrong by 50–150 ms directly skews transient CE.

**Bench rigs (garage-grade):**
- **Pressure (COP, CIP):** fast ball/solenoid valve between a regulated reservoir
  (~15–30 psi) and the sensor port; a GPIO pulse marks t=0; **burst-log** the step.
  ≥10–20 reps.
- **Thermal (COT, CIT):** mechanized plunge boiling↔ice (tight lower bound) **and** an
  air-to-air heat-gun step (representative of the gas medium). t=0 via microswitch/GPIO.
  Fit in °C. Keep the NTC in rating; measure at **in-circuit divider current**
  (self-heating). ≥10–20 reps.

**Marker record (new code on BOTH sides).** A `type:"m"` record (monotonic `t_ms` +
label) emitted by firmware on a GPIO/serial trigger. It does **not** exist yet:
`parse.ts` only handles `t`/`s` and routes everything else to a log line
(`parse.ts:18-21`). Adding it requires (a) firmware emit, (b) a `MarkerRecord` in
`types.ts`, (c) a `parse.ts` branch, (d) optionally a `store.ts` collection.

**Output:** a per-sensor `responseTime ± 95 % CI`, stored as alignment config, **tagged
with the operating point it was measured at** (COT's response time is flow-dependent —
record idle vs the representative-boost value used).

---

## Part 3 — The alignment buffer

Built in the host (`tools/ovgt-telemetry/src/align.ts`), then mirrored in firmware.

```ts
export class DelayLine {                 // ring buffer of (t_s, v); ≥ responseTimeMax of history
  constructor(capacity: number);         // 64 @ 10 Hz ≈ 6.4 s ≫ responseTimeMax
  push(tS: number, v: number): void;
  valueAt(tTargetS: number): number | undefined;  // linear interp between bracketing
                                                   // samples; undefined if older than the
                                                   // buffer (startup/gap). tTarget is
                                                   // always ≤ newest (delay_i ≥ 0), so no
                                                   // forward extrapolation.
}
export interface SensorResponseTime { cit: number; cip: number; cot: number; cop: number; }  // seconds

// HOST trigger: each 10 Hz telemetry frame (all four channels share one frame t_ms;
// there is no separate "COT arrival" event in the NDJSON — that is a firmware concept,
// CotSensor::update gate at ovgt.cpp:184). Returns:
//   a CE fraction, OR -1 (the compressorEfficiency sentinel: BR ≤ 1 or rise < 1 K — the
//   genuinely-undefined region), OR undefined (buffers lack history yet).
export function alignedCE(tNowS: number, cotNow: number,
                          lines: { cop: DelayLine; cip: DelayLine; cit: DelayLine },
                          responseTime: SensorResponseTime): number | -1 | undefined;
//   maxRt = max(responseTime.*)
//   cop   = lines.cop.valueAt(tNowS − (maxRt − responseTime.cop))   // ABSOLUTE hPa
//   cip   = lines.cip.valueAt(tNowS − (maxRt − responseTime.cip))   // ABSOLUTE hPa
//   cit   = lines.cit.valueAt(tNowS − (maxRt − responseTime.cit))   // °C
//   BR    = cop / cip                                                // absolute/absolute ratio
//   return compressorEfficiency(BR, cit, cotNow)
```

**BR uses ABSOLUTE pressures.** `cop_hpa`/`cip_hpa` are absolute (the `Hpaa` suffix), so
`BR = cop_hpa / cip_hpa` matches the firmware exactly (`ovgt.cpp:53-54`). The delay lines
buffer the **absolute hPa** channels — **never** `boost_psi` (gauge) or `tip_psi`, which
would make BR a gauge/gauge ratio and corrupt CE.

**Notes:** linear interpolation handles the channels' differing cadences. Buffer must
exceed responseTimeMax; 64 @ 10 Hz is ample. CIT/CIP are quasi-static, so their delayed
values ≈ current values — **the alignment that actually moves CE is COP→COT**; the
four-way framework is uniform and correct, that one pair is the real work.

---

## Part 4 — Validation

CE is display-only → iterate freely offline, then port.

1. **Offline-first** (host): recompute aligned CE from a re-instrumented capture
   (despike-only filtering + burst pressure steps + marker records). Tune the response
   times and the alignment; zero control risk.
2. **Ground truth — and its limits:**
   - **Steady-state anchors** (the fixed `ce_settled`, commit 7429414) validate only the
     **steady-state limit**: at settle points all channels are quasi-static, so the delays
     move nothing and `raw ≈ aligned` *trivially*. **These say nothing about transient
     alignment** — the whole point of the feature.
   - **Transient truth (the gating dependency):** a **bench step with a fast reference
     probe** is the real validator and must be procured/specced — it is not optional.
     Settle-point-*bracketed ramps* (CE trusted at both ends) help but are **necessary,
     not sufficient**: a wrong delay can still land inside the envelope and avoid CE > 1.
     So "stays between the trusted endpoints" and "stops producing CE > 1.0" are
     **screening** checks, not proof of correct alignment.
3. **Port to firmware** (C-Next) once the offline model holds: emit `aligned_ce`
   alongside `ce_pct`/`ce_settled`. Still display-only.

---

## Known error floors (what Phase 1 does NOT fix)

State these honestly so the acceptance bar is realistic:

- **Emit-time vs acquisition-time skew.** All four inputs share one frame-level `t_ms`
  (emit instant, `ovgt.cpp:65`), but COT is read asynchronously (`ovgt.cpp:184`) and the
  pressures come from the free-running ADS round-robin — so each channel is up to ~one
  loop/COT interval (~10–100 ms) stale relative to its stamp. The delay line therefore
  aligns *emit* times with a fixed per-channel bias of that order. It is < ~10 % of a
  0.5–2 s delay (acceptable first cut) but is a real floor and is **invisible offline**
  (replay faithfully reproduces the single timestamp). Emitting **per-sensor acquisition
  timestamps** (epic Phase 0.3) removes it.
- **Frequency-dependent residual.** Delay-to-slowest is exact only at DC; fast COT
  transients keep a residual skew. The lag-compensation enhancement (below) and a faster
  probe (Phase 2) are what close this.
- **Flow-dependent COT response time.** A thermocouple responds faster at high gas flow,
  so a single fixed value mis-sets the delays during exactly the varying-flow transients
  the feature targets. Phase 1 ships one fixed value (tagged with its operating point);
  residual transient error is **not bounded** until it is flow-scheduled (future).
- **Post-de-EMA COP response time assumption** (Part 2) — must be burst-measured, not
  assumed 0.

## Config, decisions, future enhancements

- **Response times are measured constants per sensor**, stored as config (firmware:
  alongside `cotSettleCfg`; host: JSON), each tagged with its measurement operating point.
- **Resolved (2026-06-23):** scheme = delay-faster-to-slowest; quantity = response time
  (lag), not settle-time; COT in-vehicle, others on the bench; BR from absolute hPa;
  name it `responseTime`, not `tau`.
- **Open (epic):** Phase-1 numeric acceptance tolerance (needed to give §4 a pass/fail
  beyond the CE > 1 screen); `ce_pct` firmware gating.
- **Future — lag-compensate COT** (`T_true ≈ T_meas + responseTime_COT·dCOT/dt`, the
  correct first-order inverse) to shrink both the alignment delays and the residual common
  delay, at the cost of amplified COT noise. Baseline ships delay-to-slowest.
- **Future — flow-schedule COT's response time** by mass-flow/boost.
