# Exhaust Brake Design (v1)

## Summary

Add a VGT-based exhaust brake. When the transmission torque converter is locked
and the engine is making no power (throttle and load both at/near 0%), close the
turbine vanes under closed-loop control to hold a target exhaust backpressure
(default 60 psi), generating engine braking on descents. The brake yields the
actuator back to the normal boost controller the instant the driver applies
throttle, on stale CAN data, or when a hardware-protection backpressure ceiling
is exceeded.

Trigger signals (torque converter lockup, throttle, engine load) are decoded
from the J1939 bus — the receive path, currently a stub, is implemented as part
of this feature.

## Vehicle Context & Safety Limit (researched)

Target engine: 2004 HPCR 5.9L Cummins (24-valve ISB).

The limiting failure mode for exhaust braking on this engine is **exhaust valve
float**: drive (pre-turbo) pressure pushes the exhaust valve off its seat once it
exceeds what the valve springs can hold closed, causing loss of braking and risk
of valvetrain/valve-to-piston damage. It is *not* a turbo or head-gasket limit.

- Industry-standard exhaust brake for this engine (PacBrake PRXB) regulates to a
  **constant ~60 psi** backpressure and ships with **60 psi heavy-duty exhaust
  valve springs**. Running ~60 psi safely **requires those springs**.
- Cited hard ceiling is **~65 psi** (valve float).
- The Cummins "~1-3 psi" figure seen in forums is the *post-turbo tailpipe*
  restriction spec — a different measurement, not relevant to this limit.

> **HARDWARE DEPENDENCY:** A target >= ~55 psi requires 60 psi exhaust valve
> springs installed. Truck spring status is currently unconfirmed, so the target
> and ceiling are exposed as tunable constants with this warning in the code.

Our feedback sensor, `turbineInputPressureHpa` (turbine *inlet* = pre-turbo drive
pressure), measures exactly the pressure acting on the exhaust valves, so it is
the correct signal both to control on and to limit against.

## Architecture

New module mirroring the existing `BoostController` static-class pattern.

### ExhaustBrakeController (src/control/exhaustBrakeController.h/.cpp)
- Static class: `Initialize()`, `bool update()`.
- Owns the brake state machine, the PI control loop, and all guard logic.
- `update()` returns `true` when the brake is active and has written
  `appData.actuatorDemandedPosition` this cycle; `false` otherwise.
- Returns `false` immediately when `manualMode` is active, leaving the existing
  serial bench-test override and boost behavior untouched.

### Actuator arbitration (src/domain/ovgt.cpp)
Exactly one controller owns the actuator per cycle:

```cpp
J1939::Loop();                                   // moved BEFORE controllers so RX is fresh
bool braking = ExhaustBrakeController::update();  // writes demanded position only when active
if (!braking) BoostController::update();          // boost owns the vanes otherwise
```

### J1939 receive decode (src/sensors/j1939.cpp)
Implement the existing `j1939Sniff()` stub. Decode the PGN from the 29-bit ID,
then populate `appData` for the two PGNs we need:

| PGN | Acronym | Rate | Field | Source | Decode |
|-----|---------|------|-------|--------|--------|
| 61442 | ETC1 | 10 ms | `torqueConverterLockupStatus` | SPN 573, byte 1 bits 3-4 | `(buf[0] >> 2) & 0x03` |
| 61443 | EEC2 | 50 ms | `acceleratorPedalPercent` | SPN 91, byte 2 | `buf[1] * 0.4` |
| 61443 | EEC2 | 50 ms | `engineLoadPercent` | SPN 92, byte 3 | `buf[2]` (1 %/bit) |

Receive timestamps (`lastEec2RxMs`, `lastEtc1RxMs`) are recorded in the callback
for staleness detection. Torque converter lockup is a 2-bit status; only `0b01`
(engaged) counts as locked (`00` not engaged, `10` error, `11` not available).

PGN extraction: `pf = (id >> 16) & 0xFF; ps = (id >> 8) & 0xFF;`
`pgn = (pf >= 240) ? ((pf << 8) | ps) : (pf << 8);` — both 61442 (0xF002) and
61443 (0xF003) have PF >= 240, so PS is included.

## AppData Changes

Add:
- `uint8_t  torqueConverterLockupStatus;`  // SPN 573, raw 2-bit (0-3)
- `uint8_t  acceleratorPedalPercent;`      // SPN 91, scaled 0-100
- `uint8_t  engineLoadPercent;`            // SPN 92, 0-250 (1 %/bit)
- `volatile uint32_t lastEec2RxMs;`        // millis() of last EEC2 receive
- `volatile uint32_t lastEtc1RxMs;`        // millis() of last ETC1 receive
- `bool     exhaustBrakeActive;`           // telemetry / debug line

(`volatile` on the timestamps because they are written in the CAN receive
callback context.)

## Control Logic (evaluated each 10 ms control cycle, in priority order)

The control block in `ovgt::loop()` is gated to ~10 ms (`loopElapsed < 10`), so
the PI loop runs at `dt = 0.01s` (computed from `millis()` for correctness).

1. **Manual override** — if `manualMode`, reset integral, return `false`.
2. **Stale-CAN failsafe (guard 4)** — if `now - lastEec2RxMs > CAN_STALE_TIMEOUT_MS`
   OR `now - lastEtc1RxMs > CAN_STALE_TIMEOUT_MS`, reset integral, return `false`.
3. **Instant release (guard 1)** — if `acceleratorPedalPercent > 0` OR
   `engineLoadPercent > 0`, reset integral, return `false`. Driver always wins.
4. **Engage condition** — `torqueConverterLockupStatus == 0b01` AND throttle == 0
   AND load == 0. No engage debounce (instant release makes a spurious engage
   cheap). If not met, return `false`.
   **Vane convention (verified on-truck):** `0%` = fully CLOSED = max
   backpressure; `68%` = fully OPEN = no restriction. Building backpressure
   means driving the position DOWN toward closed.

5. **Backpressure ceiling cutoff (guard 3)** — if measured TIP >
   `BACKPRESSURE_CEILING_PSI`, relieve by OPENING the vanes:
   `actuatorDemandedPosition = VANE_OPEN_PERCENT`, set fault flag, reset
   integral, return `true`. Overrides PI.
6. **PI loop** — otherwise (positive error = below target = need MORE
   backpressure = MORE closure = LOWER vane position):

   ```
   tipGaugePsi    = turbineInputPressureHpa * 0.0145038
   error          = TARGET_BACKPRESSURE_PSI - tipGaugePsi
   travel         = VANE_OPEN_PERCENT - VANE_CLOSED_PERCENT
   integralTerm   = clamp(integralTerm + Ki * error * dt, 0, travel)  // anti-windup
   closure        = Kp * error + integralTerm
   vane           = VANE_OPEN_PERCENT - closure
   actuatorDemandedPosition = clamp(vane, VANE_CLOSED_PERCENT, VANE_OPEN_PERCENT)
   ```

   Anti-windup clamps the integral *contribution* (in vane-% units) to
   `[0, travel]`, so it cannot wind past full travel while the vanes are
   saturated closed.

   No derivative term: TIP is noisy and D would jitter the vanes.

## Constants (defaults; all tunable on the truck)

| Name | Default | Rationale |
|------|---------|-----------|
| `TARGET_BACKPRESSURE_PSI` | 60 | PacBrake PRXB setpoint — **requires 60 psi valve springs** |
| `BACKPRESSURE_CEILING_PSI` | 65 | Valve-float limit; hardware protection cutoff |
| `VANE_OPEN_PERCENT` | 68 | Fully open = mechanical limit / relief position |
| `VANE_CLOSED_PERCENT` | 0 | Fully closed = max backpressure (but see note below) |
| `CAN_STALE_TIMEOUT_MS` | 250 | 5x EEC2 period, 25x ETC1 period |
| `Kp` | 0.5 (%vane/psi) | Starting estimate — tune on truck |
| `Ki` | 0.3 (%vane/psi/s) | Starting estimate — tune on truck |
| integral contribution clamp | `[0, travel]` | Anti-windup, in vane-% units |

> **On-truck finding (2026-06-13):** with `VANE_CLOSED_PERCENT = 0` the actuator
> only physically reaches ~9–11% before stopping short, and backpressure stays
> ~1–2 psi even motoring down an 8% grade. Suspected mechanical/actuator travel
> issue (actuator not completing closure, or linkage). Revisit
> `VANE_CLOSED_PERCENT` and gains after the hardware is sorted.

A prominent code comment documents the valve-spring dependency next to
`TARGET_BACKPRESSURE_PSI` / `BACKPRESSURE_CEILING_PSI`.

## Data Flow

```
J1939 RX (CAN2) -> j1939Sniff -> appData.{torqueConverterLockupStatus,
                                          acceleratorPedalPercent,
                                          engineLoadPercent,
                                          lastEec2RxMs, lastEtc1RxMs}
                                            |
turbineInputPressureHpa (TIP sensor) -------+--> ExhaustBrakeController::update()
                                                  -> appData.actuatorDemandedPosition
                                                  -> Actuator -> PWM/CAN to vanes
```

## Debug / Telemetry

Add brake state and backpressure-vs-target to the existing 1 Hz debug line in
`ovgt::handleDebug()` (e.g. `Brake:ON/off Tgt:60 TIP:xx.x psi`), reading the new
AppData fields. `exhaustBrakeActive` drives the indicator.

## Testing

Decision and control logic are pure functions -> native unit tests (no hardware)
under `test/`:
- PGN byte-decode correctness for ETC1 (lockup 2-bit extraction) and EEC2
  (throttle scaling, load).
- Engage/release truth table, including instant-release precedence over engage.
- Stale-timeout disengage.
- Ceiling cutoff forcing vanes open and overriding the PI output.
- PI convergence to target and integral anti-windup against a stubbed plant.

Implementation follows TDD (failing test first).

## Out of Scope (v1 / future)

- Engage debounce (deliberately omitted; revisit if spurious engages observed).
- RPM gain-scheduling (decode EEC1 SPN 190, schedule Kp/Ki) — structure leaves a
  seam for it.
- Driver enable/disable switch and brake "aggressiveness" levels.
- Logging brake events to FRAM.

## Sources

- Cummins Diesel Forum — "Maximum allowable exhaust pressure":
  https://www.cumminsforum.com/threads/maximum-allowable-exhaust-pressure.2575150/
- Cummins Diesel Forum — "Exhaust Backpressure?":
  https://www.cumminsforum.com/threads/exhaust-backpressure.2550952/
- Diesel Truck Resource — "Cummins exhaust backpressure":
  https://www.dieseltruckresource.com/forums/performance-accessories-2nd-gen-only-91/cummins-exhaust-backpressure-78281/
- PacBrake PRXB kit (2004.5-2005 5.9L ISB), C44033:
  https://pacbrake.com/c44033-direct-mount-4-inch-prxb-exhaust-brake-kit-for-20045-2005-dodge-ram-cummins-59l-isb.html
- Glacier Diesel Power — PacBrake PRXB 5.9L ISB 24V listing:
  https://www.glacierdieselpower.com/products/pacbrake-in-line-mount-4-inch-prxb-high-performance-exhaust-brake-kit-for-dodge-ram-1998-5-cummins-5-9l-isb-24-valve-engine-w-47re-automatic-transmission-c44063
