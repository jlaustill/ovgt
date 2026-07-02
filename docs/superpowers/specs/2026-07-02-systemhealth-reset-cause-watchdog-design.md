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

## Host pipeline constraint (drives the Phase 1 shape)

Phase 1 is **firmware-only** — no changes to `tools/ovgt-telemetry`. Two verified
facts about the current host tool shape the design:

- `store.ts:43` inserts `{ ...sample, ts, sessionId }` — it **spreads the whole
  sample**, so any new field added to a `type:"t"` record is **stored in Mongo
  automatically**, no host change needed.
- `parse.ts:18-21` routes `type:"t"` → telemetry, `type:"s"` → settle, and **any
  other `type` → `"log"` (discarded, not stored)**. A new `type:"boot"` record
  would therefore be **dropped** by today's parser.

**Consequence:** in Phase 1 the reset/health information rides as **fields on the
regular `"t"` record** (which auto-stores), not as a separate boot record. The
dedicated `type:"boot"` record, the full crash-report text in Mongo, and the TUI
banner are Phase 2 (host tool).

## Scope

### In scope (Phase 1 — firmware-only, flashable ASAP)

- Reset-cause capture from the IMXRT `SRC_SRSR` register.
- A boot counter persisted across reset in a retained GPR register.
- New fields on the 10 Hz `"t"` record: `reset_cause`, `boot_count`, `crash`,
  `pg`, `vin_mv`, `loop_us_max`, `loop_us_avg`, `setup_ms`.
- On-chip IMXRT WDOG1 watchdog with compile-time disable.
- Loop-timing profiling so the 2 s watchdog timeout can later be right-sized from
  data.

### Deferred (Phase 2 — host tool + evidence-driven, separate spec)

- Dedicated `type:"boot"` record: `parse.ts`/`store.ts` ingest, full
  `crash_report` text into Mongo, TUI reset banner + last-reset-cause/boot-count
  display, and a loop-timing analysis view.
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
  sequence and records the inter-feed interval for loop-timing stats.
- **`sampleSupply()`** — reads the Vin ADC pin, tracks **minimum since last
  telemetry emit** (fast dips otherwise fall between 10 Hz samples).
- **Accessors** for telemetry assembly: `resetCause()`, `bootCount()`,
  `crashPresent()`, `loopMicrosMax()`, `loopMicrosAvg()`, `supplyMillivolts()`,
  `setupMillis()`. The max/avg/min accessors reset their accumulators on read so
  each 100 ms window is independent.

Register access (`SRC_SRSR`, `WDOG1_*`, `SRC_GPR*`, ADC) uses C-Next
bit-indexing. Register addresses are referenced from the Teensy core
`imxrt.h`; where C-Next cannot reach a C macro directly, a minimal `extern`
declaration or per-register C++ shim bridges it.

### 2. C++ interop shim (minimal, unavoidable)

`CrashReport` is a C++ core object (has `operator bool` and stream semantics),
not reachable from C-Next. Phase 1 needs only the **presence** of a crash
(`crashPresent()` → the `crash` boolean field); the full report text stays as the
existing `Serial.println(CrashReport)` (captured in the serial `.log`), and is
routed into Mongo in Phase 2. If reading `operator bool` from C-Next proves
awkward, a one-line C++ shim `bool crashReportPresent()` bridges it. The watchdog
stays pure-register in C-Next (no `WDT_T4` C++ template instantiation).

### 3. Watchdog

- IMXRT **WDOG1**, timeout **~2 s** (the main loop runs every 10 ms, so the
  margin is ~200×) as a **conservative starting value** to be tightened from the
  loop-timing data (§4) once we know the real worst-case inter-feed interval.
- Started early in `SystemHealth.init()`, then **fed after each blocking
  `Initialize()` call** in `ovgt::setup()` (ADS, MAX31856, FRAM, CAN can each
  block) and once per loop tick thereafter.
- A loop hang > timeout → hardware reset → the next boot's `reset_cause` reports
  `wdog`. Converts the silent ~44 s-dead observed on 2026-07-02 into a ~2 s
  auto-recovery.
- **Compile-time togglable** (`OVGT_WATCHDOG_ENABLED`) so it can be disabled and
  reflashed immediately if it ever causes a boot loop.

### 4. Telemetry additions (`ovgt.cpp`, fields on the `"t"` record)

Added to the existing 10 Hz `"t"` record (all auto-store via `store.ts` spread):

| Field | Type | Meaning |
| ----- | ---- | ------- |
| `reset_cause` | string | `por` / `brownout` / `wdog` / `wdog3` / `lockup` / `jtag` / `pin` / `unknown`. Constant per session; emitted every sample so it is robust to dropped samples and trivially queryable. |
| `boot_count` | int | Retained-register boot counter. |
| `crash` | bool | A Teensy `CrashReport` was present on this boot (full text in serial log; into Mongo in Phase 2). |
| `pg` | bool | Power-good input (`appData.pgFault`). |
| `vin_mv` | int | Supply millivolts, minimum since last emit; `-1` until the Vin divider is present. |
| `loop_us_max` | int | Maximum inter-feed (100 Hz tick) interval, µs, since last emit. |
| `loop_us_avg` | int | Mean inter-feed interval, µs, since last emit. |
| `setup_ms` | int | Total `setup()` duration. Constant per session; characterizes the boot-time budget (the blocking inits a tight boot-time watchdog would trip on). |

**Loop-timing rationale:** the watchdog's 2 s is a guess. `loop_us_max` /
`loop_us_avg` are stored every 100 ms, so a `$max` / `$avg` / histogram over a
drive reveals the true worst-case and typical inter-feed interval. That is the
data needed to drop the timeout from 2 s to a realistic value (likely far less)
without risking a false-trip boot loop.

### 5. Host tool

**No Phase 1 changes.** The new `"t"` fields flow through `parse.ts` (already a
`type:"t"` record) and `store.ts` (spreads the sample) unmodified. All host-tool
work — the dedicated boot record, full crash text in Mongo, TUI banner, and a
loop-timing analysis view — is Phase 2.

## Data flow (Phase 1)

```
boot:  SRC_SRSR ──► SystemHealth.init() ──► resetCause / bootCount
       CrashReport ──► crashPresent() ──► crash bool
                       (full text)   ──► Serial.println  (serial .log only; Mongo in Phase 2)
loop:  100 Hz tick interval ──► loopMicrosMax/Avg ──┐
       PG_PIN ──► appData.pgFault ─────────────────┤
       Vin ADC ──► SystemHealth.sampleSupply() ────┼─► ovgt "t" record ─► Serial JSON
                                                    │      ─► host parse (type:"t") ─► store spread ─► Mongo
       reset_cause / boot_count / crash / setup_ms ─┘        (no host change)
       (every tick) SystemHealth.feed() ──► WDOG1
```

## Testing

### Native (`pio test -e native`, host, no hardware)

- **Reset-cause decode**: feed synthetic `SRSR` bit patterns → assert the correct
  cause string (one case per bit, plus multiple-bits-set precedence).
- **Boot counter**: increment logic given a starting retained value.
- **Loop-timing tracker**: max- and avg-since-emit compute correctly over a
  sequence of intervals and reset after each emit.
- **Vin min-tracker**: minimum-since-emit and the `-1` "no divider" sentinel.
- **`"t"` record JSON shape**: extend `test_json` to cover the new fields.

Any newly tested `src/` file is added to the `[env:native]` `build_src_filter`
in `platformio.ini`.

### On-hardware

- Force each reset type and confirm `reset_cause` in Mongo: power-cycle → `por`;
  a deliberate `> timeout` blocking delay in a test build → `wdog`; a software
  reset call → software/`unknown` as applicable.
- Trigger a null-dereference in a test build → confirm `crash:true` in Mongo (and
  the report text in the serial log).
- Confirm the watchdog does **not** fire during a normal cold boot (setup feeding
  works).
- Sanity-check `loop_us_avg` ≈ 10 000 µs at idle and that `loop_us_max` tracks
  transient spikes.

## Risks and mitigations

| Risk | Mitigation |
| ---- | ---------- |
| WDT too aggressive → boot loop | 2 s timeout (200× loop margin); feed through setup's blocking inits; `OVGT_WATCHDOG_ENABLED` compile-time kill switch; tighten only from logged data. |
| C-Next cannot reach an `imxrt.h` register macro | `extern` declaration or a minimal per-register C++ shim; the bit-logic stays in C-Next. |
| Boot counter lost on full power loss | Acceptable — that loss *is* the brownout signal. FRAM-backed counter deferred until FRAM lands. |
| `crash` field misses the full report | Full text remains in the serial `.log` (existing println); Phase 2 routes it into Mongo. The `crash` bool + `reset_cause` are enough to classify the reset. |
| Emitting constant fields every sample wastes bandwidth | `reset_cause`/`boot_count`/`setup_ms` are a few bytes at 115200 baud — negligible; every-sample emission is robust to dropped samples and keeps the parser unchanged. |

## Open decisions carried into implementation

- Exact spare ADC pin for Vin and the divider ratio (owner selects based on free
  pins; both are compile-time constants).

## Related

- Memory: `project_engine_bay_heat_soak` (revised — lockups now look
  electrical/firmware, not thermal), `project_cnext_migration`,
  `project_deployment_environment`, `feedback_no_runtime_config`,
  `project_fram_status`.
