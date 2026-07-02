# systemHealth.cnx — Reset-Cause Capture & Watchdog Robustness (Phase 1)

**Date:** 2026-07-02
**Status:** Approved design — ready for implementation planning
**Author:** Joshua Austill (with Claude)

## Motivation

On 2026-07-02 a full drive (session `linux-Oryx-Pro-1782996682136`, label "New
Tranmission Tuning") showed the Teensy 4.1 **reboot twice** mid-session, proven by
the monotonic `t_ms` uptime counter resetting to zero:

| Wall time | MCU temp | State | Recovery |
| --------- | -------- | ----- | -------- |
| ~18:17:46 | 79 °C    | idle / spool, vane 22% | silent multi-second gap → `t_ms` reset |
| ~19:27:48 | 57.5 °C  | idle / cooldown | silent multi-second gap → `t_ms` reset |

Forensic findings from the telemetry:

1. **It died with the lights on.** The last sample before each hang is
   indistinguishable from the hundreds before it — no sensor spike, no NaN, no
   mode change, no control excursion. The current telemetry set has **zero
   predictive signal** for these resets.
2. **Not thermal.** Both resets happened at *idle*, and the board had just
   *survived* 27 psi / 766 °C hard pulls minutes earlier at only 65 °C. One reset
   was at 57.5 °C — a Teensy does not thermal-fault there.
3. Leading hypotheses, none confirmable from what we log today: **12 V supply
   brownout/transient** (crank, key-cycle, alternator, transmission-ECU switching)
   or a **firmware hang** (blocked I²C/SPI/CAN).

The firmware already *has* most of the diagnostic signal — it just never reaches
MongoDB:

- `CrashReport` is printed on boot (`ovgt.cpp:98`) as plain serial text, but the
  telemetry tool's parser keeps only JSON lines, so it is discarded.
- `PG_PIN` power-good is read into `appData.pgFault` (`ovgt.cpp:173`) but never
  emitted.
- DWT cycle-timing counters already accumulate per loop section but are not
  emitted.

**This is an observability seam, not missing sensors.** The cheapest robustness
win is to close that seam and add fast auto-recovery.

## Goal

Make every Teensy reset **self-documenting in MongoDB**, and add **fast
auto-recovery**, so the next drive tells us electrical-vs-firmware instead of
guessing. Strictly additive — **zero change to control behavior or tuning.**

## Scope

### In scope (Phase 1 — flashable ASAP)

- Reset-cause capture from the IMXRT `SRC_SRSR` register.
- A structured **boot record** emitted into the telemetry stream.
- A boot counter persisted across reset in a retained GPR register.
- `pgFault`, Vin (supply voltage), and loop-timing fields added to the 10 Hz
  telemetry record.
- On-chip IMXRT WDOG1 watchdog with compile-time disable.
- Host-tool changes to parse, store, and surface the new records.

### Deferred (Phase 2 — separate spec, after Phase 1 data)

- Safe-vane-on-boot behavior (the vane flops to ~82% during a reboot window).
- The evidence-driven power/hardware brownout fix (hold-up capacitance, wiring,
  regulator).
- Controller relocation out of the engine bay and clock reduction
  (600 → 396 MHz).

### Non-goals

- No control, PID, or tuning changes.
- **No runtime config** — the WDT enable and Vin pin are compile-time `#define`s
  (driving hazard; tuning is edit-and-flash only).

## Hardware assumptions

- `PG_PIN` is a **digital power-good** flag only (no analog voltage today).
- The owner will add a **resistor divider from Vin (12 V) to a spare Teensy
  analog pin** so real supply voltage can be sampled. The firmware is written
  ready for it: the Vin ADC pin and divider ratio are compile-time constants, and
  the `vin_mv` field reports `-1` until the divider is present and the pin is
  configured.

## Architecture

### 1. `src/domain/systemHealth.cnx` (new C-Next module)

Public scope `SystemHealth`:

- **`init()`** — called first thing in `ovgt::setup()`:
  - Read and decode `SRC_SRSR` reset-cause bits (power-on, brownout/LVD,
    watchdog WDOG1, watchdog WDOG3, lockup, JTAG, external pin), then
    write-1-to-clear them so the next reset starts clean.
  - Increment a boot counter stored in a **retained GPR register** (`SRC_GPR*`).
    Survives watchdog/software resets; a full power-loss zeroing it is itself a
    brownout signal.
  - Start the watchdog (see §3).
- **`feed()`** — called once per 100 Hz loop tick; performs the WDOG1 service
  sequence.
- **`sampleSupply()`** — reads the Vin ADC pin, tracks **minimum since last
  telemetry emit** (fast dips otherwise fall between 10 Hz samples).
- **Accessors** for telemetry assembly: `resetCause()`, `bootCount()`,
  `maxLoopMicros()`, `supplyMillivolts()`.

Register access (`SRC_SRSR`, `WDOG1_*`, `SRC_GPR*`, ADC) uses C-Next
bit-indexing. Register addresses are referenced from the Teensy core
`imxrt.h`; where C-Next cannot reach a C macro directly, a minimal `extern`
declaration or per-register C++ shim bridges it.

### 2. C++ interop shim (minimal, unavoidable)

`CrashReport` is a C++ core object (has `operator bool` and stream semantics),
not reachable from C-Next. A tiny C++ helper:

```cpp
// returns true if a crash report was present; writes a truncated,
// JSON-safe string into buf
bool crashReportToString(char* buf, size_t n);
```

This is the **only** C++ interop in Phase 1 — the watchdog itself stays
pure-register in C-Next, avoiding `WDT_T4` C++ template instantiation.

### 3. Watchdog

- IMXRT **WDOG1**, timeout **~2 s** (the main loop runs every 10 ms, so the
  margin is ~200×).
- Started early in `SystemHealth.init()`, then **fed after each blocking
  `Initialize()` call** in `ovgt::setup()` (ADS, MAX31856, FRAM, CAN can each
  block) and once per loop tick thereafter.
- A loop hang > 2 s → hardware reset → the next boot record reports
  `reset_cause: "wdog"`. Converts the silent ~44 s-dead observed on 2026-07-02
  into a ~2 s auto-recovery.
- **Compile-time togglable** (`OVGT_WATCHDOG_ENABLED`) so it can be disabled and
  reflashed immediately if it ever causes a boot loop.

### 4. Telemetry additions (`ovgt.cpp`)

**Boot record** — emitted once, in `setup()` after `SystemHealth.init()`:

```json
{"type":"boot","t_ms":12,"reset_cause":"wdog","boot_count":37,
 "fw":"<build id>","crash":true,"crash_report":"<truncated>"}
```

`reset_cause` is one of: `por`, `brownout`, `wdog`, `wdog3`, `lockup`, `jtag`,
`pin`, `unknown`. `crash` is false and `crash_report` omitted when no crash
report is present.

**Per-sample health** — folded into the existing 10 Hz `"t"` record:

- `pg` — power-good boolean (`appData.pgFault`).
- `vin_mv` — supply millivolts, minimum since last emit (`-1` until divider
  present).
- `loop_us_max` — maximum main-loop duration (µs) since last emit, from the DWT
  cycle counter.

### 5. Host tool (`tools/ovgt-telemetry`)

- **`parse.ts`** — accept `type:"boot"` lines and the new `"t"` fields; keep
  ignoring non-JSON.
- **`store.ts`** — persist boot records to the `telemetry` collection tagged by
  `type`, so they are queryable alongside samples (`{type:"boot"}` filter).
- **`app.tsx`** (TUI) — surface **last reset cause + boot count**, and show a
  visible **reset banner** when a boot record arrives, so a lockup is obvious
  live rather than only in post-hoc forensics.

## Data flow

```
boot:  SRC_SRSR ──► SystemHealth.init() ──► resetCause/bootCount
       CrashReport ──► crashReportToString() ──┐
                                               ├─► ovgt boot record ─► Serial JSON ─► host parse ─► Mongo (type:"boot")
loop:  DWT timing ──► maxLoopMicros() ─────────┤
       PG_PIN ──► appData.pgFault ─────────────┤
       Vin ADC ──► SystemHealth.sampleSupply() ─► ovgt "t" record ─► Serial JSON ─► host parse ─► Mongo (type:"t")
       (every tick) SystemHealth.feed() ──► WDOG1
```

## Testing

### Native (`pio test -e native`, host, no hardware)

- **Reset-cause decode**: feed synthetic `SRSR` bit patterns → assert the correct
  cause enum/string (one case per bit, plus multiple-bits-set precedence).
- **Boot counter**: increment logic given a starting retained value.
- **Loop-timing tracker**: max-since-emit resets correctly after each emit.
- **Vin min-tracker**: minimum-since-emit and the `-1` "no divider" sentinel.
- **Boot-record JSON shape**: extend `test_json` to cover the boot record and the
  new `"t"` fields (escaping of `crash_report`).
- **Host**: `parse.ts` test for a `type:"boot"` line; `store` round-trip test for
  a boot record (requires local MongoDB).

Any newly tested `src/` file is added to the `[env:native]` `build_src_filter`
in `platformio.ini`.

### On-hardware

- Force each reset type and confirm the boot record's `reset_cause` in Mongo:
  power-cycle → `por`; a deliberate `>2 s` blocking delay in a test build →
  `wdog`; a software reset call → software/`unknown` as applicable.
- Trigger a null-dereference in a test build → confirm `crash:true` and the
  report text land in Mongo.
- Confirm the watchdog does **not** fire during a normal cold boot (setup
  feeding works).

## Risks and mitigations

| Risk | Mitigation |
| ---- | ---------- |
| WDT too aggressive → boot loop | 2 s timeout (200× loop margin); feed through setup's blocking inits; `OVGT_WATCHDOG_ENABLED` compile-time kill switch. |
| C-Next cannot reach an `imxrt.h` register macro | `extern` declaration or a minimal per-register C++ shim; the bit-logic stays in C-Next. |
| Boot counter lost on full power loss | Acceptable — that loss *is* the brownout signal. FRAM-backed counter deferred until FRAM lands. |
| Boot record too large / breaks 10 Hz timing | Emitted once at boot only, not in the hot path; `crash_report` truncated to a fixed cap. |
| Host tool drops unknown `type` | Explicit `parse.ts`/`store.ts` handling + a test guarding it. |

## Open decisions carried into implementation

- Exact spare ADC pin for Vin and the divider ratio (owner selects based on free
  pins; both are compile-time constants).
- `crash_report` truncation length (start ~200 chars; revisit if useful detail is
  cut).

## Related

- Memory: `project_engine_bay_heat_soak` (revised — lockups now look
  electrical/firmware, not thermal), `project_cnext_migration`,
  `project_deployment_environment`, `feedback_no_runtime_config`,
  `project_fram_status`.
