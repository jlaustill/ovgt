# Compressor-Efficiency (CE) Accuracy — Design

## Problem

CE (`Ti·(Pr^0.286−1)/(To−Ti)`) divides by the compressor temperature rise
`To−Ti`, so any error in that rise blows up the result. Three things corrupt it:

1. **Probe lag** — the COT thermocouple (sheathed K-type) responds slowly, so on
   transients CE reads garbage (>100%): COT hasn't caught up to the real air temp.
2. **Quantization** — COT/CIT are stored as whole-°C `int16`, throwing away the
   sensors' real resolution; `To−Ti` is coarse at small rises.
3. **Heat-soak** — at idle/low-flow the probe reads conducted housing heat, not
   compression, so CE there is meaningless (no real compression to measure).

This design fixes accuracy **in software, no hardware, no control-loop changes**,
and adds an in-firmware **probe-response measurement** so a faster probe can be
characterized and compared empirically.

## Scope decision (from brainstorming)

- **Build now (Approach A):** float resolution + a slope-detection core driving two
  outputs — a CE "settled" trust-flag and a probe time-constant (τ) measurement.
- **Document only:** the faster-probe hardware swap (B1/B2) as the follow-on for
  live/transient CE.

## Architecture

```
COT sensor (float) ─┐
boost (COP−CIP) ────┤→ cotSettle core ─┬→ settled flag ──→ CE `~` marker (display/trust)
                    │   (slope detect)  └→ τ measurement ──→ "COT settle:" report line
CIT sensor (float) ─┴→ compressorEfficiency() (unchanged math)
```

One new pure module (`cotSettle`) feeds both outputs from shared slope tracking.
Control loops (`boostController`, `exhaustBrake`) read none of these fields — they
are untouched.

## Part 1 — Resolution: `CIT` & `COT` → `float` °C

`AppData.compressorInputTempC` and `compressorOutputTempC` change from `int16_t` to
`float` (TIT stays `int16` — display-only, not in CE).

**Consumers to update (audited blast radius):**
- `src/sensors/adcSensors.cpp` — CIT write (processResult case 2): store the
  Steinhart-Hart result as float (drop `(int16_t)` cast).
- `src/sensors/cotSensor.cpp` — COT write: store `readThermocoupleTemperature()` as
  float (drop `(int16_t)` cast).
- `src/domain/ovgt.cpp` debug line — `CIT:%dC`/`COT:%dC` → `%.1f`; CE call drops the
  `(float)` casts (already float).
- `src/sensors/j1939.cpp` — **audit during implementation**: if CIT/COT are
  transmitted on a PGN, rescale the encode. If not, no change.

No other readers (brake/boost use pressures + lockup/throttle/load, not these temps).

## Part 2 — `cotSettle` core (`src/sensors/cotSettle.{h,cpp}`, pure, native-tested)

Fed once per **new COT conversion (~10 Hz)** — NOT the 1 Hz debug tick. (At 1 Hz a
fast exposed probe with τ < 0.5 s would be invisible.) `CotSensor::update()` returns
`bool` (true when it wrote a fresh sample); the main loop feeds `cotSettle` on new
samples with the measured elapsed `dt`.

~10 Hz is the **MAX31856 hardware ceiling**, not a software choice: continuous-mode
conversions take ~82–98 ms (≈10–12 Hz) and each value is already averaged over that
window, so COT dynamics faster than ~0.1 s are unrecoverable from this sensor path
regardless of code. Set **averaging=1 + 60 Hz filter** for the full ~12 Hz. This
resolves τ ≳ 0.2 s well — sheathed (~seconds): trivial; typical exposed junction
(~0.3–1 s): ~10–30 samples across the rise, 63%-crossing interpolated to ~±0.1 s. An
ultra-fine probe (τ < ~0.2 s) reads only as "≈0.1–0.2 s" — still unmistakably faster,
so the B1 decision is unaffected.

```cpp
struct CotSettleConfig {
    float cotSlopeFlatCperS;    // |dCOT/dt| below this = "COT not moving"  (~0.3)
    float boostSlopeFlatPsiPerS;// |dBoost/dt| below this = "boost steady"   (~0.5)
    float settledSeconds;       // sustained-flat duration for the CE flag    (~2.0)
    float minStepC;             // min |COT change| to accept a τ measurement (~3.0)
    uint16_t maxBufferSamples;  // ring-buffer cap (slowest settle * rate)   (~512)
};

struct CotSettleResult {
    bool  settled;            // CE-trust flag: COT + boost both flat, sustained
    bool  measurementReady;   // true for the ONE sample a τ measurement completes
    float tauSeconds;         // valid when measurementReady
    float settleSeconds;      // valid when measurementReady (time to slope-flat)
    float stepC;              // measured COT change, signed (valid when ready)
};

// state holds slope history, the arm/measure event machine, and the COT ring buffer
CotSettleResult cotSettleStep(CotSettleState &state, const CotSettleConfig &cfg,
                              float cotC, float boostPsi, float dtSeconds);
```

**Slope tracking:** each call computes `dCOT/dt` and `dBoost/dt` from the previous
sample. (Float COT keeps these clean.)

**Settled flag:** when `|dCOT/dt| < cotSlopeFlatCperS` AND
`|dBoost/dt| < boostSlopeFlatPsiPerS`, accumulate time; once it reaches
`settledSeconds`, `settled = true`. Any out-of-band sample resets the accumulator.

**τ measurement — boost as the fast reference clock.** Because boost (pressure
sensor) settles in ~0.1 s, the instant boost goes flat the air temperature has
essentially reached its new value; COT's continued drift after that is *pure probe
lag*. Event machine:

1. **IDLE→ARMED:** boost transitions moving→flat. Record `t0`, `cotStart`; begin
   buffering `(elapsed, cotC)` each sample.
2. **ARMED:** keep buffering. Abort back to IDLE if boost leaves the flat band
   (the step wasn't clean) or the buffer fills (`maxBufferSamples`).
3. **ARMED→DONE:** COT slope reaches flat. If `|cotEnd − cotStart| ≥ minStepC`:
   - `settleSeconds = elapsed at this sample`
   - `target = cotStart + 0.632·(cotEnd − cotStart)` (sign-aware: works for a
     rising or falling step)
   - walk the buffer for the first crossing of `target`, linearly interpolate
     between the bracketing samples → `tauSeconds`
   - set `measurementReady = true`, return the result, reset to IDLE.
   - If the step was below `minStepC`, discard (noise) and reset to IDLE.

## Outputs / display (`ovgt.cpp` glue)

- **CE marker:** CE still prints whenever BR > 1.0 (no blanking). Append `~` when
  `!settled`: `CE:64%` (trust) vs `CE:163%~` (transient). Future Mongo logger emits
  `settled` as a boolean for automatic filtering.
- **τ report:** on `measurementReady`, print one line (like the fault prints, not on
  the periodic line):
  `COT settle: tau=3.8s t_settle=11.2s dT=+9.4C`

## Part 3 — Faster probe (documented, NOT built)

Captured as the follow-on for live/transient CE:
- **B1 — exposed-junction fast K-type:** drop-in on the **existing COT MAX31856**
  channel; only the probe changes. Rides on Part 1's float resolution. Lowest
  friction. (User already has one from the junk pile.)
- **B2 — fast NTC thermistor / RTD:** kills lag *and* quantization and reuses the
  CIT thermistor pipeline, BUT **all 8 ADS1115 channels are currently in use**
  (boost, CIP, CIT, TIP / TOP, oil-P, oil-T, lift-pump), so B2 needs added ADC
  capacity (3rd ADS1115). This makes **B1 the practical hardware path.**

The Part 2 τ measurement is exactly the **B1 go/no-go tool**: it produces the hard
numbers to justify (or not) the swap, and a real τ that could later feed an optional
software lag-compensation (`T_true ≈ T_meas + τ·dCOT/dt`) if live CE is ever wanted
without relying on the faster probe alone.

## Testing

Native Unity, `test/test_cot_settle/`:
- **Settled flag:** sustained-flat series → `settled` after `settledSeconds`; a
  mid-series slope spike resets it; boost-moving-but-COT-flat → not settled.
- **τ extraction:** feed a synthetic first-order step `cot(t)=end−(end−start)e^(−t/τ)`
  with known τ; assert returned `tauSeconds` within tolerance, and `stepC` correct.
- **Abort paths:** boost leaves flat band mid-measure → no measurement; sub-`minStepC`
  step → discarded.
- `compressorEfficiency` tests unchanged (still pass).

Then build for `teensy41`; on-truck: confirm CE shows `~` on tip-ins and clean on
cruise, and that a tip-in→cruise prints a `COT settle:` line with a plausible τ.

## Out of scope

- Faster-probe hardware swap (B1/B2) — documented above, separate effort.
- Software lag-compensation (Approach C) — noted as a future option the τ enables.
- Mongo logger integration — when it lands, it consumes `settled` + the temps; no
  change needed here.
- TIT resolution — stays `int16` °C (not in CE).

## User workflow (the payoff)

Drive on the **sheathed** probe and note the `COT settle:` τ values (expect a few
seconds). Swap in the **exposed** junk-pile probe, drive similar conditions, compare
τ (likely 5–20× smaller). Hard data for the B1 decision — and the satisfying reveal
of the sheathed probe's actual number.
