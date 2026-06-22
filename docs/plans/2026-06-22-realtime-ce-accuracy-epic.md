# EPIC — Real-Time Compressor Efficiency (CE) via Sensor Time-Alignment

**Status:** Proposed (epic / umbrella). Spawns per-phase design+plan pairs.
**Created:** 2026-06-22
**Builds on:** `2026-06-16-ce-accuracy-design.md` / `-plan.md` (the cotSettle trust-flag + τ-measurement). That work explicitly deferred "the faster-probe hardware swap as the follow-on for live/transient CE" — **this epic is that follow-on.**

---

## Vision

Today CE is only trustworthy at rare **settle points** (3 in a 23-minute drive). The goal of this epic is **CE you can trust continuously — ideally in real time** — by making all four CE inputs represent the *same instant*, then iterating the hardware so the correction needed shrinks toward zero.

CE is **telemetry/display only** (`ovgt.cpp:74-75`); no control loop reads it. So every software step here can be **iterated offline on logged raw data** (the `tools/ovgt-telemetry` replay path) with **zero risk to firmware control behavior**. That is the central enabler.

---

## The core insight

CE blends a pressure ratio with a temperature rise:

```
CE = idealRise / actualRise = [ CIT_K · (BR^0.286 − 1) ] / ( COT_K − CIT_K )
     where BR = COP / CIP
```

Every input must be sampled at the same physical moment, or CE divides *this* moment's pressure ratio by *an earlier* moment's temperature rise. That time-skew is what produces garbage CE (>100%) during transients.

Classifying the four inputs by signal dynamics collapses the problem:

| Input | Signal | Sensor | Path | Constrains transient CE? |
|-------|--------|--------|------|--------------------------|
| **CIP** | quasi-static (≈ambient) | ADS1115 ch1 | adcSensors | **No** — always settled |
| **CIT** | quasi-static (≤0.53 °C/s) | NTC (GM 25037225) | adcSensors | **No** — always settled |
| **COT** | fast & frequent (gas to 21 °C/s) | K-type / MAX31856, τ≈0.5–2 s | cotSensor | **Yes — the slow link** |
| **COP** | **fast & frequent** | ADS1115 ch0 | adcSensors | **Yes — the rich signal ("gold mine")** |

**Therefore the entire transient-CE problem reduces to time-aligning the two fast movers, COP and COT.** CIP and CIT don't move during a transient, so their absolute timing contributes no skew — they are off the critical path (characterize for completeness, don't gate on them).

**Two levers, which compound:**
1. **Software (Phase 1):** match settle time at the CE stage — filter/delay COP to COT's effective time constant so BR and the temp rise share one group delay. Gives **semi-accurate transient CE** on today's hardware.
2. **Hardware (Phase 2):** shrink COT's τ (faster probe, e.g. RIFE). As COT speeds up, the COP alignment lightens → more of COP's fast data survives as valid CE → **approach real-time CE.**

**Sharp interaction (package deal):** COP currently rides the same α=0.01 sensor-layer EMA (~0.5 s) as the rest of ADS1, which *accidentally* lands near COT's lower τ bound and is part of why CE works at all today. Stripping that EMA (the despike-only direction in [[project_sensor_filter_layering]]) **without** adding CE-stage alignment will *desync* COP from COT and make transient CE worse. Fast raw COP logging and matched-τ alignment must land together.

---

## Phase 1 — Software time-alignment → *semi-accurate* transient CE

> Lever #1. "Learn the timing of each current sensor, then adjust the timing to get a semi-accurate CE value." All offline-first on logged raw data; no control risk.

### Story 1.1 — Raw fast-signal logging (prerequisite)
Ensure COP (and COT) are logged with **despike-only** sensor-layer filtering so alignment can be fitted/applied in post. Either reduce the ADS EMA (α≈0.2–0.5 + outlier clamp) or log a raw COP channel alongside the smoothed one. Ties directly to [[project_sensor_filter_layering]].
- **Done when:** a drive capture contains COP at its true sensor bandwidth (not ~0.5 s-EMA-throttled), verifiable by `dCOP/dt` distribution widening vs drive-2.

### Story 1.2 — Sensor timing characterization
Measure the effective time constant / group delay of each fast mover.
- **COT:** firm up the drive-2 estimate (τ≈0.5–2 s from fast rising edges; cooldowns measure *gas* dynamics, not the sensor — don't use them). A bench step (heat-gun on/off with a reference) gives the cleanest τ.
- **COP:** characterize the ADS conversion + any residual EMA group delay (analytic + step fit).
- **CIP/CIT:** document as quasi-static / non-constraining; in-vehicle τ is unmeasurable (no fast stimulus — same reason CIT timing can't be measured, see [[project_cot_probe_tau_baseline]]). Optional bench step, off critical path.
- **Done when:** a documented τ (and method) per fast mover + the EMA group delay.

### Story 1.3 — Offline CE alignment model (host replay)
In `tools/ovgt-telemetry`, implement matched-time-constant alignment: low-pass COP (and CIP) to COT's effective τ before computing CE. Compute an **aligned transient CE** and compare it against the trusted settle-point CE.
- **Recommended method:** matched first-order low-pass on COP to COT's τ (simple, robust). *Rejected:* COT deconvolution (noise amplification); pure delay (COT isn't a pure delay).
- **Done when:** aligned transient CE matches settle-point CE within the agreed tolerance at comparable operating points (same BR / CIT), and shows no >100% garbage through transients.

### Story 1.4 — Port validated alignment to firmware
Once the offline model is validated, implement the alignment in firmware (C-Next, consistent with the migration) and emit an **aligned CE** field alongside the existing gated `ce_pct` / `ce_settled`. Still display/telemetry only.
- **Done when:** live aligned CE on the truck tracks the offline model within tolerance; control loops untouched.

---

## Phase 2 — Hardware/sensor iteration → *approach real-time* CE

> Lever #2. "Iterate on the hardware/sensors and hopefully shrink the adjustment to get closer to real-time CE."

### Story 2.1 — Faster COT probe + co-located timing test
Mount the candidate faster probe (RIFE Hi-AT thermistor / exposed-junction) **co-located with the current K-type at the compressor outlet**, capture matched fast steps, and fit τ for both. This **also resolves the long-deferred NTC-vs-thermocouple timing comparison** that CIT can never provide (see [[project_cot_probe_tau_baseline]]).
- **Done when:** measured candidate τ vs the K-type baseline (~0.5–2 s), from matched steps.

### Story 2.2 — Re-tune alignment for the faster probe
With a smaller COT τ, reduce the COP alignment filter and re-validate transient CE. Quantify how much closer to real-time we get (residual alignment lag).
- **Done when:** residual alignment τ and transient-CE error are quantified for the new probe.

### Story 2.3 — Iterate to "real-time"
Repeat 2.1–2.2 across probe candidates / mounting until the residual alignment is small enough that CE is effectively real-time for the use case.
- **Done when:** CE error during normal driving transients is within the real-time target with negligible alignment lag.

---

## Sequencing & dependencies

```
1.1 raw logging ──► 1.2 characterize ──► 1.3 offline align/validate ──► 1.4 firmware port
                                              │
                                              ▼
2.1 faster probe + co-located timing ──► 2.2 re-tune align ──► 2.3 iterate to real-time
```
Phase 1 delivers value (semi-accurate transient CE) on current hardware and is independent of RIFE availability. Phase 2 reuses the exact alignment/validation machinery from 1.3, just with a smaller COT τ.

## Decisions to confirm
- **"Semi-accurate" tolerance target** for Phase 1 (e.g. aligned transient CE within X% of settle-point CE)?
- **Raw COP logging vs light-EMA + characterized group delay** — storage/bandwidth acceptable for raw, or characterize the filter instead?
- **RIFE availability/ETA** — gates Phase 2 start (Phase 1 does not wait on it).

## Risks
- COT not cleanly first-order → matched single-τ low-pass under/over-corrects. Mitigate: bench step characterization; consider 2-pole model if residuals demand it.
- Truck electrical noise on the small COP/CIP signals corrupts alignment. Mitigate: despike + light filter, validate on logged data.
- Heat-soak at low flow still makes CE meaningless regardless of timing (pre-existing, per `2026-06-16-ce-accuracy-design.md`) — alignment fixes *timing*, not the no-real-compression regime.

## Definition of done (epic)
Live, display-grade CE that is trustworthy through normal driving transients (not just settle points) on the chosen probe, with the time-alignment validated offline-then-firmware and the residual lag characterized.
