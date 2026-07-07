# MCU Health J1939 Frame — Design Spec

- **Date:** 2026-07-06
- **Status:** Approved design (pre-implementation)
- **Repos affected:** `jlaustill/J1939` (canonical definition + codec), OVGT (reference
  firmware impl), OSSM, OCT (consumer firmware), OCT logger / telemetry host (decoder)

## Objective

Every MCU in the fleet (OVGT, OSSM, OCT — all Teensy 4.x / i.MX RT1062) broadcasts its
own health on the J1939 bus, starting with **MCU die temperature**, so that a single
listener can collect all of them keyed by **Source Address (SA)**.

No standard J1939 SPN represents a *controller MCU die temperature*, so a proprietary
frame is used. The frame is designed once as a fleet-wide contract rather than per-board.

## Transport: Proprietary B PGN

- **PGN `65500` (`0xFFDC`)** — Proprietary B (PDU2, broadcast), priority 6.
- **Rate:** 1000 ms (health is slow-moving).
- **CAN ID:** `0x18 FF DC <SA>` — priority 6, PGN `0xFFDC`, source = the sending MCU's SA.
- **Why PDU2/broadcast:** one fixed PGN; each node transmits it with its *own* SA in the
  CAN ID, so a listener collects `SA → payload` directly. No destination addressing and no
  address-claim handshake required.
- **Why not Proprietary A (`0xEF00`):** destination-specific point-to-point — the wrong
  shape for "every MCU broadcasts to whoever's listening."
- **Collision check (2026-07-06):** OSSM already uses `65280 (0xFF00)` and `65281 (0xFF01)`
  for its config command/response protocol; the truck's factory bus uses only standard
  `0xFExx` PGNs. `0xFFDC` is unused across OVGT/OSSM/OCT and the factory bus.

## Payload (8 bytes, little-endian)

| Bytes | Field       | Type / Encoding                                   | N/A sentinel |
|-------|-------------|---------------------------------------------------|--------------|
| 0–1   | MCU die temp| u16, 0.03125 °C/bit, offset −273 (std J1939 temp) | `0xFFFF`     |
| 2     | Reset cause | u8 enum (below)                                   | `0xFF`       |
| 3–4   | Boot count  | u16                                               | `0xFFFF`     |
| 5–6   | Uptime      | u16 **minutes** (~45.5 days before wrap)          | `0xFFFF`     |
| 7     | Reserved    | `0xFF`                                            | —            |

- Temp encoding matches OVGT's existing temperature SPNs (`j1939EncodeTemperatureRaw`) so
  one decoder handles every temperature on the bus.
- Uptime is minutes (not seconds) so a u16 does not wrap during a realistic drive
  (seconds would wrap at ~18.2 h).

### Reset-cause enum (shared)

Reused verbatim from the Teensy i.MX RT1062 `SRSR` decode already in OVGT
(`src/domain/systemHealthLogic.cnx`); the same silicon is on all three boards, so the enum
is genuinely fleet-universal:

| Code | Meaning   | Code | Meaning     |
|------|-----------|------|-------------|
| 0    | unknown   | 5    | wdog3       |
| 1    | por       | 6    | tempsense   |
| 2    | lockup    | 7    | jtag        |
| 3    | pin       | 8    | csu         |
| 4    | wdog      |      |             |

- `0` = decode ran but cause was not recognized.
- `0xFF` (byte-2 sentinel) = this MCU does not implement reset-cause decode — distinct
  from `0` = decoded-but-unknown.

### Sentinel convention

`0xFF` / `0xFFFF` per field = "not available / not implemented on this MCU." Sentinels are
**per-field**, so a board that only knows some fields still emits a valid frame.

## Canonical definition home

- Lives in **`jlaustill/J1939`** on the **`main` (v2.x)** line and is published to the
  PlatformIO registry. It ships **ahead of and independent of** the `v3-codec-protocol`
  rewrite — this feature does not wait on v3.
- New library module (working name **`J1939McuHealth`**) providing, as a sans-I/O pure
  unit:
  - the PGN constant (`0xFFDC`) and rate,
  - the reset-cause enum,
  - **encode** `{ tempC, resetCause, bootCount, uptimeMinutes }` → 8 bytes (applying
    sentinels for absent fields),
  - **decode** 8 bytes → the same struct with per-field validity.
- **Consumers:** OSSM and OCT already declare `jlaustill/J1939` in `platformio.ini`; they
  pick this up on a version bump. **OVGT hand-rolls J1939** today (no lib dep) — it is
  still the reference implementation, and **it will adopt the `jlaustill/J1939` dependency**
  (decided 2026-07-06) so the whole fleet shares one J1939 codepath for this frame.

## Component boundaries

- **`J1939McuHealth` (library):** constants + enum + encode/decode only. No CAN I/O, no
  time source. Fully host-testable.
- **Each MCU firmware:** populate the struct from its own health source, call `encode`,
  transmit the 8 bytes at 1000 ms on its CAN peripheral with its own SA.
- **Listener (OCT logger + telemetry host):** on RX of PGN `0xFFDC`, `decode` and store
  `health[SA]` (a small map, one record per source address).

## Data flow

```
MCU health source ─► encode(struct) ─► 8 bytes ─► CAN TX (SA) ─► bus
bus ─► listener RX (PGN 0xFFDC) ─► decode ─► health[SA] ─► log / display
```

## Rollout order

1. **J1939 lib:** add `J1939McuHealth` on `main`, host unit tests, bump version, publish.
2. **OVGT:** reference implementation — transmit at 1000 ms (already has temp, reset
   cause, boot count via systemHealth; uptime = `millis() / 60000`).
3. **OSSM, OCT:** populate available fields, sentinel the rest.
4. **Listener:** SA-keyed decode + collection in the OCT logger / telemetry host.

## Testing

- **Library (host):** encode/decode round-trip including all sentinels; boundary temps
  (−273 °C, high); every enum value; uptime wrap boundary.
- **Firmware (bus):** `candump` shows `18FFDC<sa>` at ~1 Hz; fields decode to expected
  values for each board.

## Out of scope / YAGNI

- No transport protocol / multi-packet — single 8-byte frame only.
- No health fields beyond the five above; byte 7 is reserved for growth without a new PGN.
- No dependency on or migration to the v3 codec — ships on the v2 line now; v3 may
  re-host the module later.

## Open items for the implementation plan

- Exact `J1939McuHealth` module naming and API shape to match v2 library conventions.
- Per-MCU uptime source (OSSM/OCT equivalents of `millis() / 60000`).

## Decisions log

- 2026-07-06: OVGT will **adopt the `jlaustill/J1939` library dependency** (retiring its
  hand-rolled J1939 for this frame) rather than vendor the constants — fleet shares one
  J1939 codepath.
