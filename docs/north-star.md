# OVGT North Star — Maximize Compressor Efficiency

> **This is the objective function of the entire project.** Every feature, sensor,
> filter, and control decision is in service of it. When a choice is unclear, ask:
> *does this make CE more accurate, or let us act on it?* If neither, it is not the
> priority. Read this before planning any CE-, sensor-, or control-related work.

---

## The goal

**Control the VGT vanes to keep the compressor at maximum efficiency at all times** —
extract the most boost-per-unit-energy the turbo can deliver. Compressor efficiency
(**CE**) is the signal we are ultimately trying to optimize, so **CE is the north
star**, and making it *trustworthy enough to control on* is the whole game.

CE is read-only today **only because it is not yet dependable enough to close a loop
on.** Fixing that — not adding features — is the critical path.

## What CE is

Isentropic compressor efficiency (`src/sensors/compressorEfficiency.cpp`):

```
CE = idealRise / actualRise = [ CIT_K · (BR^0.286 − 1) ] / ( COT_K − CIT_K )
     BR = COP / CIP        (both ABSOLUTE pressures; boost = COP − CIP is the gauge derivative)
     0.286 = (γ−1)/γ, air, γ = 1.4
```

Four inputs, **all vital**: **CIP** (inlet pressure), **CIT** (inlet temp), **COT**
(outlet temp), **COP** (outlet pressure). CE is a *single-instant ratio*, so the
four must describe the **same moment** or the number is meaningless.

## The central challenge: accuracy = precision **and** time-alignment

"Accurate CE" has two axes that **pull against each other**:

- **Precision** — low noise. Improved by *filtering*, which **adds lag**.
- **Time-alignment** — the four inputs at one instant. Destroyed by *lag*.

The physics says these are **not symmetric** (verified, multi-agent):
- After any reasonable filtering, sensor noise contributes only **~0.1–0.4 % CE RMS**.
- COT probe lag alone injects **+5 to +28 % CE error** during normal tip-ins
  (gas dT/dt up to 21 °C/s; noise/lag crossover at dCOT/dt ≈ 0.08–0.3 °C/s — two
  orders below real slew).
- What actually corrupts transient CE is the **difference in group delay between the
  fast movers (COP vs COT)** — matched delays keep CE within ~2 % of truth; a 1 s
  mismatch yields ~274 % garbage.

**So the objective is: match group delay across the inputs, then filter only as much
as precision genuinely needs (despike at the sensor layer; heavy smoothing only at
the display/analysis layer).** Lag is the enemy; precision is nearly free.

A nuance that reconciles "all four are vital" with "align the fast movers first":
CIP/CIT are **co-equal for accuracy** (CIT's per-°C CE leverage = `1 + rise/Tin` >
COT's at *every* operating point; CIP elasticity ~2.9 at cruise) and carry real
**diagnostic** value, but for **timing** they are **second-order** (on a real drive
CIP moved only ~0.7 psi total). They must be carried to a common timestamp for
accuracy; they need only a small fixed lag term for alignment.

## Physical realities the project must respect

1. **Vanes control the *turbine*, not the compressor directly.** Closing vanes →
   faster gas → more turbine power → more turbo speed → more boost; the compressor's
   operating point (and thus CE) is the *downstream* consequence. "Control CE" =
   "modulate turbine power to move the compressor onto a high-efficiency island."
2. **"Max CE at all times" is *constrained* maximization,** not an unconstrained
   maximand. Real constraints: surge margin, boost/torque demand (driver), EGT /
   backpressure ceiling, transient spool. The peak-CE point can also sit *away* from
   the engine brake-thermal-efficiency optimum — CE is the dominant lever for BTE,
   not identical to it. The eventual objective must state which it optimizes.
3. **CE is undefined off-boost** (BR ≤ 1: idle, cruise, overrun — much of a truck's
   life). A CE-objective controller needs a defined **fallback mode** (e.g.
   feedforward-only) where CE does not exist.
4. **There is no airflow sensor.** The compressor map's x-axis (corrected mass flow)
   and any surge-margin number require it. Realistic paths: reconstruct an
   **empirical** efficiency map from logged data + a **speed-density airflow estimate**
   (J1939 RPM + MAP + IAT + a VE map) or a broadcast J1939 MAF; detect surge by its
   **low-frequency boost oscillation on the fast COP channel**, not a flow margin.
5. **CE > 1.0 is physically impossible** and the firmware emits it *uncapped*. Treat
   any CE > 1 as a **free built-in detector** of COT/COP time-skew or non-adiabatic
   (heat-soak) operation — surface it, don't hide it.
6. **Heat-soak biases CE in magnitude, not just timing.** Housing heat into the air
   biases (COT − CIT) → CE reads low; a cold housing → CE high/>1. Needs a heat-soak
   correction or a CIT-vs-ambient trust gate *alongside* time-alignment.
7. **Hardware sample-rate floors are real.** The MAX31856 (COT) caps at ~10–12 Hz
   (~100 ms; one-shot is *slower*, not faster — you cannot outrun it by config), and
   telemetry is decimated to 10 Hz. Alignment can never be finer than the slowest
   channel's interval. A genuinely faster COT probe therefore also needs a **faster
   ADC front-end** (e.g. an NTC on the ADS path, which round-robins faster than the
   MAX31856's 100 ms), not just a faster sensing element.

## The two levers (they compound)

- **Software** — strip the sensor-layer EMA toward despike-only, match group delay
  across inputs at the CE stage, optionally lag-compensate COT
  (`T_true ≈ T_meas + τ·dCOT/dt`). Bounded by the temperature sensors' *irreducible*
  thermal mass (software fixes the pressure channels; it cannot remove a thermocouple's
  seconds-long τ).
- **Hardware** — shrink COT's τ (faster probe: exposed-junction K-type on the existing
  MAX31856, **or** an NTC on the ADS channel currently borrowed by lift-pump) and, if
  the probe outpaces the MAX31856's 100 ms floor, a faster ADC front-end. Improve
  absolute calibration (a 5 °C COT bias at a 60 °C rise ≈ 8 % CE error).

## The gating prerequisite

**Before any alignment or control: a rock-solid, dependable, accurate way to measure
each sensor's response time / time-drift** (the user's repeatedly stated requirement).
The dependable primary is a **bench step-response against a fast reference probe**
(in-vehicle, the stimulus timing is never cleanly known; COP↔COT cross-correlation
mixes sensor lag with real thermodynamic phase and is only a heuristic). This is
**Phase 0** of the epic.

## Definition of "done enough to control on"

CE is dependable when its **residual error and phase lag during real driving
transients are inside the control loop's tolerance** — i.e. CE noise below ~half the
planned control perturbation and CE phase lag well inside the loop time constant.
Those numbers come from the chosen control bandwidth; they are the acceptance bar the
whole measurement+alignment program is working toward.

## Sequence

`measure sensor time-drift (Phase 0) → time-align inputs (software) → validate aligned
CE against trusted ground truth → only then close a CE control loop`, with the faster
COT probe (hardware) pursued in parallel. Software work is **control-risk-free**
(CE is display-only) but **not free in sequencing**: existing logs are EMA-filtered
and 10 Hz-decimated and cannot support alignment fitting, so firmware
re-instrumentation + a fresh drive is a hard gate before offline iteration.

## Guardrails

- Every step serves CE accuracy or the ability to act on it. If it does neither,
  it is not the priority.
- Despike at the sensor layer; smooth heavily only at the display/analysis layer
  (preserve response time in what gets logged and fed to the gate/loop).
- Never work around a sensor/driver limitation downstream — fix it upstream
  (per repo policy), e.g. the MAX31856 config or the ADC front-end, not a hack.
- CE > 1.0 and the heat-soak regime are signals, not noise — use them.

## See also

- **Plan:** `docs/plans/2026-06-22-realtime-ce-accuracy-epic.md` (the phased epic)
- **Foundation:** `docs/plans/2026-06-16-ce-accuracy-design.md` (cotSettle gate + τ)
- **Tooling:** `tools/ovgt-telemetry/` (offline replay — where software alignment is iterated)
