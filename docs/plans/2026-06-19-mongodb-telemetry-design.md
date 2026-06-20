# OVGT Telemetry → MongoDB + Live TUI — Design Spec

**Date:** 2026-06-19
**Status:** Design (awaiting review → implementation plan)
**Supersedes:** Part B of `2026-06-19-cnext-doc-review-then-mongodb-telemetry.md` (Part A, the C-Next audit, is complete).

## Goal

Stream Teensy telemetry as newline-delimited JSON (NDJSON) over USB serial at 10 Hz to a host terminal tool that (1) shows it live, (2) logs every sample to MongoDB, and (3) sends tuning commands back. This replaces the "paste serial text and grep it" workflow and enables matched-operating-point analysis (manual vs auto), settled-CE-only views, BPR-error and τ trends, and **offline replay of the `~` settled-CE gate**.

## Context (current firmware)

- `src/domain/ovgt.cpp` — 100 Hz main loop (`loopElapsed >= 10`). `handleDebug()` emits a **1 Hz human-readable line** (`BR:… Boost:… BPR:… …`); `handleSerial()` parses tuning commands; `cotSettleStep()` is the `~` gate (inputs COT, boost psi, dt → `settled` flag + τ/settle/step measurements when `measurementReady`).
- `include/AppData.h` — `AppData appData`, the canonical telemetry struct.
- C-Next migration is in progress (`src/display/actuator.cnx` is the pilot, transpiled to `.hpp`/`.cpp` and called from C++). The JSON builder is the next C-Next module. The C-Next agents guide (just audited) is the authority for `.cnx` syntax.

## Architecture

```
 ┌────────── Teensy 4.1 firmware ──────────┐         ┌────────────── Host (TypeScript) ──────────────┐
 │ 100 Hz loop                              │  USB    │ serialport (115200) → readline                │
 │   every 100 ms: Json.build(telemetry) ──┼─serial─→│   JSON.parse → route by "type":               │
 │   gate measurement: Json.build(settle) ──┼─NDJSON─→│     telemetry → OpenTUI live view + Mongo     │
 │ handleSerial(auto/bpr/kp/ki/<pos>) ←─────┼─cmds────┤     settle    → Mongo settle_events           │
 └──────────────────────────────────────────┘         │   keypresses → serial.write("bpr 1.5\n")      │
                                                       └────────────────────────────────────────────────┘
```

No web server, no sockets, no auth, single device + single host, USB serial only.

Two units, coupled only by the wire protocol:
1. **Firmware** — a reusable C-Next `Json` builder scope + a 10 Hz emit integrated into the loop. The existing tuning-command handler is unchanged.
2. **Host** — a TS CLI/TUI using **OpenTUI** + **serialport** + the **mongodb** driver.

## Resolved decisions

- **Wire format: NDJSON-only.** The 1 Hz human-readable data line is *replaced* by 10 Hz NDJSON. Boot/diagnostic/ack text lines (CrashReport, "Setup complete", "BPR target = …") stay plain text and are shown in the TUI's **log pane**, not parsed as telemetry.
- **Rate: fixed 10 Hz** (every 100 ms), one telemetry object per tick. Settle events are emitted as their own NDJSON object when the gate produces a measurement.
- **Session label: TUI keypress.** A keypress sets/changes the active session's label live; the label is stamped on the session doc (and new samples carry the current `sessionId`).
- **Sequencing: C-Next `Json` builder first** — no C++ `snprintf` stepping stone.

## Component 1 — Firmware C-Next `Json` builder scope

**File:** `src/domain/json.cnx` → transpiled to `json.hpp`/`json.cpp`, called from `ovgt.cpp`.

A deterministic, heap-free, MISRA-clean NDJSON builder using a fixed buffer + cursor:

- **State (scope members):** `string<N> buf;` and a `u32 cursor;`. `N` is proven from a worst-case dummy object (all fields at max width) plus margin — a compile-time constant.
- **API (`scope Json`):**
  - `begin()` — reset `cursor`, write `{`.
  - `addUint(string key, u32 value)`, `addInt(string key, i32 value)` — write `"key":<digits>` via manual base-10 conversion (single-byte writes at the cursor; the audit confirmed `s[i] <- c` byte writes and `s[i]` byte reads).
  - `addFloat(string key, f32 value, u8 decimals)` — fixed-decimal formatting by scaling to an integer (e.g. ×100 for 2 decimals) and placing the decimal point; no `printf`, no heap.
  - `addBool(string key, bool value)` — `true`/`false`.
  - `addStr(string key, string value)` — quoted; values are controlled identifiers (mode names), so no escaping needed in v1.
  - `end()` — write `}`, terminate; expose the finished `buf` (and length) for the caller to `Serial.println`.
- **Bounds-checked:** every `addX` checks remaining capacity against `N`; on overflow it stops writing and sets a `truncated` flag rather than overrunning (C-Next is memory-safe — a slice/byte write past the end is a compile/again-runtime guard, never UB).
- **No padding** — clean numeric tokens so values land in Mongo as real numbers, not strings.
- Adding a telemetry field later = one `addX` call.
- Candidate to graduate into a reusable c-next `json` module.

**Integration (`ovgt.cpp`):**
- Change the cadence from the 1 Hz `debugTimer` to **10 Hz** (100 ms). Simplest: drop `debugTimer`/`debugFlag` and gate emission on the existing 100 Hz loop with a counter (emit every 10th iteration), or a 100 ms `elapsedMillis`.
- `handleDebug()` is rewritten: build the telemetry object with `Json` and `Serial.println` it (no more `snprintf` pretty line).
- The settle line (`"COT settle: …"`) becomes a **settle NDJSON object**, emitted when `ceRes.measurementReady`.

## Component 2 — On-wire protocol

**Device → host:** NDJSON, one JSON object per `\n`-terminated line. Two object types, discriminated by `"type"`:

- **Telemetry** (10 Hz): `{"type":"t","t_ms":…,"mode":"auto|manual|brake", …fields…}`
- **Settle** (on gate measurement): `{"type":"s","t_ms":…,"tau_s":…,"settle_s":…,"step_c":…}`

Any line that is **not valid JSON** (boot banner, CrashReport, command acks) is treated by the host as a diagnostic log line (shown, not stored as telemetry).

**Host → device:** the existing line commands, `\n`-terminated, unchanged: `auto`, `params`, `bpr <v>`, `kp <v>`, `ki <v>`, `<0-100>` (vane %). TUI keypresses map to these.

### Telemetry field list

Source = `AppData` + computed values from `handleDebug()` + `BoostController` + the gate. Units explicit in the key.

**Core (parity with the old line + gate context):**
| key | type | meaning |
|---|---|---|
| `t_ms` | u32 | firmware `millis()` — for exact dt between samples |
| `mode` | str | `auto` / `manual` / `brake` |
| `cop_hpa`, `cip_hpa` | u16 | compressor out / in pressure (hPa abs) |
| `boost_psi` | f32 | (COP − CIP) gauge, psi |
| `br` | f32 | COP / CIP (compressor pressure ratio) |
| `tip_psi` | f32 | turbine inlet (drive) pressure, psi |
| `bpr` | f32 | TIP / boost (drive ratio — the controlled variable) |
| `bpr_target`, `kp`, `ki` | f32 | live controller config (replay/tuning context) |
| `cit_c`, `cot_c` | f32 | compressor inlet / outlet temp |
| `tit_c` | i16 | turbine inlet temp |
| `ce_pct` | f32 | compressor efficiency, 0–100 (`-1` when undefined — keeps a fixed schema) |
| `ce_settled` | bool | the `~` gate flag |
| `dem_pct`, `pos_pct` | u8 | actuator demanded / reported position |
| `brake` | bool | `exhaustBrakeActive` |

**Gate-replay extras (raw gate state — needs surfacing from `cotSettleStep`):**
| key | type | meaning |
|---|---|---|
| `cot_slope_c_s` | f32 | COT slope (°C/s) the gate computed |
| `boost_slope_psi_s` | f32 | boost slope (psi/s) the gate computed |
| `settle_timer_s` | f32 | how long the gate has seen "flat" |

These three live in `CotSettleState`/intermediate vars, not in `CotSettleResult` — a small firmware change exposes them (build step 2).

**Optional (cheap, include if room):** `tot_c`, `oil_temp_c`, `oil_psi`, `lift_pump_psi`, `actuator_load`, `actuator_status`; J1939 `tcc_lockup`, `accel_pct`, `engine_load_pct`; diagnostics `mcu_temp_c`, `clk_mhz`.

## Component 3 — Host TS CLI/TUI

**Location:** `tools/ovgt-telemetry/` inside the ovgt repo (confirmed at review — cohesion, one clone).

**Stack:** TypeScript, OpenTUI (terminal UI), `serialport`, `mongodb`. No web server/sockets.

- **Serial:** auto-detect the Teensy (VID:PID `16C0:0483`, as `logToTee.sh` does) or accept `--port`; 115200; readline by `\n`.
- **Parse:** `try { JSON.parse(line) }` → on success route by `type` (`t` → telemetry, `s` → settle); on failure → log pane. RegExp/guards per SonarCloud prefs.
- **Live view (OpenTUI):** current telemetry formatted (a richer version of the old pretty line) + gate state (settled?/timer/slopes) + BPR vs target error. A log pane for diagnostic lines. (Sparklines/history = YAGNI for v1; Mongo/Compass is for analysis.)
- **Tuning keymap → serial writes:** e.g. `[`/`]` BPR target ∓0.05, `<`/`>` kp, `,`/`.` ki, digit-entry → vane %, `a` auto, `p` params, `l` set/relabel session (prompt). Exact keymap finalized in the plan.
- **Mongo:** insert each telemetry sample and settle event; create a session doc on connect; update session `label` on the relabel keypress; set `endTs` on exit.

## Component 4 — MongoDB schema (database `ovgt`)

- **`ovgt.telemetry`** — one doc per sample = parsed telemetry object + host `ts` (Date) + `sessionId`.
- **`ovgt.settle_events`** — one doc per gate measurement = parsed settle object + `ts` + `sessionId`.
- **`ovgt.sessions`** — one doc per host connect: `{ _id: sessionId, startTs, endTs, label, firmwareGitSha?, host, notes }`. The host owns session stamping; firmware stays session-agnostic.
- **Indexes:** `telemetry` and `settle_events` on `{ sessionId: 1, t_ms: 1 }`.

The `mongodb` MCP is available, so analysis queries can be run directly to assist the `~`-gate tuning.

## Data flow

1. Loop (100 Hz) → every 100 ms: `Json` builds the telemetry line → `Serial.println` → host readline → `JSON.parse` → { TUI update, Mongo `telemetry` insert }.
2. Gate `measurementReady` → `Json` builds a settle line → same path → Mongo `settle_events`.
3. TUI keypress → `serial.write("bpr 1.5\n")` → firmware `handleSerial` → `BoostController::setBprTarget`.

## Error handling

- **Firmware:** `Json` is bounds-checked (overflow → `truncated` flag, never overruns); serial is fire-and-forget (no host attached is harmless).
- **Host:** serial disconnect → reconnect loop; malformed line → log pane, continue; Mongo insert failure → log + continue (never block the live view); all `JSON.parse` guarded.

## Testing

- **Firmware `Json` builder:** C-Next `.test.cnx` unit tests (per the c-next conventions just audited) — each `addX`, the overflow/`truncated` path, float fixed-decimal formatting, and a full-object round-trip against an expected string.
- **Host:** parser unit tests (recorded NDJSON → expected objects; malformed → skipped); a fake-serial fixture replaying a recorded log; Mongo insert against a local/Atlas-local deployment.
- **Integration / replay:** firmware emits → host parses → Mongo has the doc; replay a captured `ovgt-*.log`-style NDJSON file through the host offline (this is also the `~`-gate replay capability).

## Build order

1. Firmware C-Next `Json` builder scope + unit tests.
2. Surface the gate internals (`cot_slope`, `boost_slope`, `settle_timer`) from `cotSettleStep`.
3. Wire 10 Hz telemetry + settle NDJSON into `ovgt.cpp` (replace the pretty line / 1 Hz timer).
4. Host: serial read + parse + OpenTUI live view (no Mongo yet) — verify against the truck.
5. Host: Mongo insert + sessions.
6. Host: tuning keymap + session-label keypress.

## Out of scope (YAGNI)

Web server, websockets, browser UI, auth, multi-device, in-TUI historical charts, config files, JSON string-escaping beyond controlled identifiers. Analysis happens in Mongo (queries / Compass), not the TUI.

## Decisions confirmed at review (2026-06-19)

- **Host tool location:** `tools/ovgt-telemetry/` in this repo. ✓
- **Surfacing the gate internals** (`cot_slope` / `boost_slope` / `settle_timer` from `cotSettleStep`) is approved as build step 2. ✓
