# J1939 full-bus logging — design

**Date:** 2026-07-03
**Status:** approved (brainstorming), pending spec review
**Repo:** ovgt (Teensy 4.1 firmware + `tools/ovgt-telemetry` host)

## Purpose

Capture the engine + transmission J1939 broadcast on OVGT's CAN2 bus, time-aligned into the
existing 10 Hz telemetry stream, so we can test two falsifiable hypotheses against real drive
data. Firmware only *logs raw signals*; all modeling/validation happens offline in MongoDB.

This spec covers **instrumentation to capture the data**. Hypothesis validation is a follow-on
analysis once ≥1 drive is logged (acceptance criteria below).

### H1 — Boost is set by exhaust *energy*, not backpressure
Turbine power ∝ ṁ_exhaust · EGT · (1 − PR^−0.28). We construct ṁ_exhaust offline via
speed-density from RPM + intake-air density (boost + intake temp), with fuel energy proxied by
`% load × RPM` (no fuel-rate PGN is broadcast — see Findings). EGT we already measure (`tit_c`).
**Prediction:** the "same 10 psi backpressure → wildly different boost" scatter collapses once
indexed on exhaust energy `E = ṁ_exhaust · EGT` (plus vane position). Pass = low-scatter,
single-valued surface / small unstructured residuals; fail = boost still smears after accounting
for E.

### H2 — The idle/coast reboots are transmission electrical brown-outs
**Prediction:** each reboot lands within ~1 s of a transmission electrical event (gear change,
TCC engage/disengage, clutch-slip spike) **and/or** a measured droop in system voltage (SPN 168).
Pass = reboots cluster on trans events / voltage sag far above chance; fail = reboots scatter
randomly vs trans activity and voltage stays flat.

## Findings that shaped this design (from `kuminz-re`, `oct`)

- OVGT's CAN2 already carries engine (SA `0x00`) and Allison trans (SA `0x03`) frames; the OCT
  gateway has already unlocked the normally-silent CM848D J1939 output. Engine vs transmission is
  separable by CAN-ID source-address low byte.
- Byte-level decodes for every target PGN already exist and are truck-validated in
  `oct/src/data/cm848-j1939-receiver.cpp` and
  `kuminz-re/firmware/CM848_S90140.06_analysis/docs/j1939-broadcasting.md` — we port, not re-derive.
- **System/battery voltage is broadcast** (VEP1 65271, SPN 168, 1 Hz) → direct brown-out evidence,
  primary H2 signal.
- **No fuel-rate PGN** in the CM848D broadcast set → H1 fuel term is `% load × RPM` (direct fuel
  rate would require proprietary CLIP polling — out of scope).

## Architecture

Three firmware layers on the existing `src/sensors/j1939.cpp` RX path (`j1939Sniff`,
`j1939.cpp:37-47`), one host change, and offline analysis.

1. **Decode layer** — extend `j1939Sniff` to decode all target PGNs (table below) into `appData`.
   New/extended decoders live in `src/sensors/j1939Decode.cpp`; each returns `(value, status)`.
2. **Signal-health layer** — every recognized PGN stamps `lastSeenMs`; every decoded SPN is
   classified `ok / na / err` from its raw bytes. A PGN not seen within its timeout is `absent`.
3. **Inventory layer** — the `j1939Sniff` `else` branch tallies every *undecoded* PGN
   `{pgn, sourceAddr, count, approxHz, lastPayload}` in a fixed-size RAM table.
4. **Emit** — decoded values ride the existing 10 Hz telemetry line (`type:"t"`,
   `ovgt.cpp:61-87`). A new **1 Hz diagnostic line** (`type:"d"`) carries signal health +
   unknown-PGN inventory.
5. **Host** — `parse.ts` routes `type:"d"`; `store.ts` inserts it into a new `j1939_diag`
   collection. Decoded telemetry fields flow into `telemetry` automatically via the existing
   `{...sample}` spread (`store.ts:43`) — no host change for those.
6. **Offline analysis** — MongoDB aggregations validate H1 (energy-collapse) and H2
   (reboot/voltage/shift correlation).

### Design principles
- **Firmware stays dumb:** decode + classify + tally only. No mass-flow/energy math, no control
  changes, no new TX. Modeling is offline until (if) H1 proves out.
- **Discover before trusting:** the inventory layer guarantees nothing on the bus is silently
  missed, including proprietary PGNs.
- **Hot line lean, diagnostics on a side channel:** 10 Hz line carries values; FF/FE/absent
  detail lives on the 1 Hz `type:"d"` line.

## Signal set (decoded → 10 Hz `type:"t"` line)

Source of truth for byte layouts: `oct/src/data/cm848-j1939-receiver.cpp`. "HAVE" = decoded today.

| JSON key | Signal | SPN | PGN (SA) | rate | serves | status |
|---|---|---|---|---|---|---|
| `engine_rpm` | Engine speed | 190 | EEC1 61444 (00) | 20 ms | H1 core | new |
| `torque_demand_pct` | Driver-demand torque | 512 | EEC1 | 20 ms | H1 | new |
| `torque_pct` | Actual engine torque | 513 | EEC1 | 20 ms | H1 | new |
| `accel_pct` | Accelerator pedal | 91 | EEC2 61443 (00) | 20 ms | H1 | HAVE (add to line) |
| `load_pct` | Engine % load | 92 | EEC2 | 20 ms | H1 fuel proxy | HAVE (add to line) |
| `intake_air_c` | Intake manifold air temp | 105 | IC1 65270 (00) | 500 ms | H1 air density | new |
| `j1939_boost_kpa` | Boost (cross-check) | 102 | IC1 | 500 ms | validate own sensor | new |
| `preturbo_kpa` | Pre-turbo/ambient pressure | 108 | AMB 65269 (00) | 1 s | context | new |
| `system_v` | System/battery voltage | 168 | VEP1 65271 (00) | 1 s | **H2 brown-out** | new |
| `coolant_c` | Coolant temp | 110 | ET1 65262 (00) | 1 s | context | new |
| `oil_c` | Engine oil temp | 175 | ET1 | 1 s | context | new |
| `oil_kpa` | Oil pressure | 100 | EFL/P1 65263 (00) | 500 ms | health | new |
| `trans_out_rpm` | Trans output shaft speed | 191 | ETC1 61442 (03) | 10 ms | H2 + driveline | new |
| `trans_in_rpm` | Trans input shaft speed | 161 | ETC1 | 10 ms | H2 | new |
| `clutch_slip_pct` | Percent clutch slip | 522 | ETC1 | 10 ms | H2 shift event | new |
| `tcc` | Torque-converter lockup | 573 | ETC1 | 10 ms | H2 | HAVE (add to line) |
| `gear_sel` | Selected gear | 524 | ETC2 61445 (03) | 100 ms | H2 shift event | new |
| `gear_cur` | Current gear | 523 | ETC2 | 100 ms | H2 | new |
| `gear_ratio` | Actual gear ratio | 526 | ETC2 | 100 ms | H2 | new |

Decoding a target frame yields all of *its* SPNs, so context signals sharing a decoded frame (e.g.
fuel temp in ET1) are emitted for free. Broadcast PGNs not in this table (EEC3 65247, DM1, AMB
extras, any proprietary frames) surface via the inventory layer and can be promoted here later.

**Per-sample validity:** a signal's 10 Hz value is emitted as its number when `status == ok`,
else JSON `null`. The FF/FE/absent distinction lives on the 1 Hz diagnostic line (a `null` in
`telemetry` means "not usable this sample; see `j1939_diag`").

## Signal-health / error detection

Per SAE J1939-71 SLOT conventions, classify each decoded SPN from its raw field:

| data length | `na` (not available) | `err` (error indicator) | else |
|---|---|---|---|
| 1 byte | `0xFF` | `0xFE` | `ok` |
| 2 byte (LE) | `raw ≥ 0xFF00` | `0xFE00 ≤ raw ≤ 0xFEFF` | `ok` |

A PGN is `absent` when `millis() − lastSeenMs > timeout`, with `timeout = max(3 × broadcastRate,
1000 ms)`. `absent` takes precedence over the last-known SPN status.

**Statuses:** `ok` | `na` | `err` | `absent` (4-state). Tracked per expected SPN in `appData`
alongside the value.

## Diagnostic message (`type:"d"`, 1 Hz)

```json
{
  "type": "d",
  "t_ms": 1234567,
  "health": { "engine_rpm": "ok", "system_v": "absent", "gear_sel": "na", ... },
  "unknown": [ { "pgn": 65247, "sa": 0, "cnt": 50, "hz": 20, "last": "FFFF03FFFFFFFFFF" }, ... ]
}
```
- `health` — object keyed by JSON signal key → 4-state status. One entry per expected signal.
- `unknown` — array of undecoded PGNs seen since boot (or since last emit): PGN, source address,
  cumulative count, approximate rate, most-recent 8-byte payload as hex. Fixed cap (e.g. 32
  entries); overflow increments a dropped counter also reported. Retaining `last` payload +
  changing-byte hints is enough to seed later reverse-engineering.

## Data model (MongoDB `ovgt`)

- `telemetry` — gains the ~16 new fields automatically (value or `null`). No schema migration.
- `j1939_diag` — **new** collection: `{ type:"d", ts, sessionId, health:{...}, unknown:[...] }`,
  1 doc/second. Index `{ sessionId:1, ts:1 }` (mirror the telemetry index in `store.ts:31`).

**Storage impact:** ~16 fields × (~8-byte value + key) ≈ +300–400 B/telemetry doc → roughly
doubles per-doc size; ~+100 MB per multi-hour drive. `j1939_diag` at 1 Hz is negligible.
Acceptable; note for retention.

## Firmware changes (insertion points)

- `src/sensors/j1939Decode.cpp` / `.h` — add `decode*` for each new SPN (return value + status);
  add `spnStatus1Byte(raw)` / `spnStatus2Byte(raw)` classifiers.
- `src/sensors/j1939.cpp` — extend `j1939Sniff` (`:37-47`): decode PGNs 61444, 65270, 65269,
  65271, 65262, 65263, 61445; extend 61442 (add 191/161/522) and 61443 (91/92 already). Stamp
  `lastSeenMs` per PGN. `else` branch → inventory tally. Add absent-timeout scan + a 1 Hz
  diagnostic builder invoked from `J1939::Loop` (`:143`).
- `appData` — add the ~16 value fields + their status enums + PGN `lastSeen` timestamps +
  inventory table.
- `src/domain/ovgt.cpp` — add the new keys to the 10 Hz JSON body (`:61-87`); add a `type:"d"`
  builder emitted at 1 Hz (reuse the `count % N` gate pattern at `:45`).

## Host changes (`tools/ovgt-telemetry`)

- `src/parse.ts` — route `type:"d"` → `{ kind: "j1939diag", doc }` (mirror the `type:"s"` branch
  at `:19`).
- `src/store.ts` — add `insertJ1939Diag(doc)` / `recordJ1939Diag(doc)` writing to `j1939_diag`
  with `sessionId` + `ts`; create its index in `connect()` (`:31`). Wire into `app.tsx`/`index.tsx`
  alongside the settle-event path.

## Testing

- `test/test_j1939_decode/` — add cases per new decoder: nominal value, `0xFF` → `na`,
  `0xFE`/`0xFExx` → `err`, plus the 2-byte boundary (`0xFEFF` err vs `0xFF00` na). Extend
  `test/test_j1939_encode/` only if any TX changes (none planned).
- Host: `tools/ovgt-telemetry` vitest — `parse.ts` routes `type:"d"`; `store.ts` writes
  `j1939_diag` (needs local MongoDB per project CLAUDE.md).
- On-truck smoke: one short drive → confirm `engine_rpm` tracks throttle, `system_v` ~13–14 V,
  gear signals change on shifts, and `unknown[]` lists the expected extras (EEC3 65247, etc.).

## Validation plan (follow-on, once data exists)

- **H1:** filter steady-state (|Δboost| < 0.5 psi/s), compute `E = ṁ_exhaust · EGT` offline
  (speed-density from `engine_rpm`, `intake_air_c`, boost; fuel proxy `load_pct × engine_rpm`);
  test whether boost is a tight single-valued function of `E` + vane position. Compare residual
  spread vs the backpressure-indexed view from `docs/analysis/2026-07-02-driving-log-review.md §5`.
- **H2:** join reboot timestamps (uptime resets, per §1 of the same doc) against `gear_cur`/
  `gear_sel` transitions, `tcc` edges, `clutch_slip_pct` spikes, and `system_v` minima in a ±2 s
  window; test clustering vs a random-time baseline.

## Out of scope

- Proprietary CLIP memory polling (direct fuel rate, DBW state) — passive listen only.
- Any firmware-side modeling, mass-flow computation, or control/feed-forward change.
- New J1939 TX from OVGT.
- Reverse-engineering proprietary unknown PGNs beyond capturing them for later.

## Open questions / risks

- **Bus availability at OVGT's tap:** engine J1939 depends on the OCT gateway having unlocked
  broadcasting. If a drive shows engine signals `absent`, that's a real finding (the health layer
  will flag it), not a bug — but confirm on the smoke drive.
- **1 Hz voltage may be too slow** to catch a fast brown-out transient; if H2 is inconclusive,
  consider whether any higher-rate voltage source exists (else accept as a known limit).
- **Inventory table sizing** vs number of distinct undecoded PGNs — start at 32 + dropped-counter.
