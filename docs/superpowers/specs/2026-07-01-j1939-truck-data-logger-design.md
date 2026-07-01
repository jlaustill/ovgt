# J1939 Truck Data Logger — Design

**Date:** 2026-07-01
**Status:** Draft (approved for spec write-up; awaiting spec review)
**Scope:** Iteration 1 — raw frame capture, storage, and query. Decode-to-named-signals is explicitly a later iteration.

## Purpose

A passive, always-on appliance that losslessly captures **every** raw J1939 frame
off the truck's CAN bus, stores it forever on-device, and lets the owner query it
over WiFi when the truck is parked. Today the OVGT controller only observes the few
PGNs it needs; this logger captures the *whole* bus so any signal can be recovered
and analyzed after the fact.

This iteration stores frames **raw only**. Decoding raw frames into named
engineering signals (via the local `j1939` MongoDB) is a deliberate follow-on step,
not part of this design.

## Requirements

### Functional
- Capture **every** J1939 frame on the truck bus, lossless, no sampling or filtering.
- Store frames on the device with real wall-clock timestamps.
- Retain data **forever** on-device (bounded only by disk; see capacity below).
- Allow querying stored frames over WiFi from a laptop on the same (home) network.
- Survive routine truck power cycles (engine start/stop, key-off) without data
  corruption.

### Non-functional
- **Passive / non-intrusive:** the logger must never transmit on, ACK, or otherwise
  disturb the truck bus (safety-relevant — the OVGT controller shares this bus).
- **Unattended:** boots and starts logging on power-up with no interaction.
- **Low maintenance:** capacity measured in years before intervention.

### Explicit non-goals (YAGNI for iteration 1)
- No decode-to-named-signal tool (later iteration).
- No web UI — query via Compass/mongosh.
- No cloud, cellular, or off-truck upload.
- No on-device analytics or dashboards.
- No bus transmit of any kind.
- No rolling-window/TTL eviction and no desktop archive/sync (keep-forever on-device
  makes these unnecessary).

## Hardware

Built on a spare Raspberry Pi 5 plus three HATs; NVMe drives are already on hand.

| Part | Choice | Notes |
|------|--------|-------|
| Compute | **Raspberry Pi 5 (8 GB)** | Runs MongoDB (arm64), built-in WiFi. On hand. |
| CAN interface | **CAN FD HAT (MCP2518FD)** | Taps CANH/CANL, **listen-only**, 250 kbps, appears as SocketCAN `can0`. |
| Storage | **NVMe HAT + 2–4 TB NVMe** | On hand. Keep-forever headroom (see capacity). NVMe, never SD — write endurance. |
| Time | **RTC HAT** (or UPS HAT with integrated RTC) | Real wall-clock timestamps with no internet on the road. |
| Power / shutdown | **Wide-input (9–36 V) automotive DC-DC + ignition-sense + supercap/UPS** | See Power & Reliability — the make-or-break subsystem. |

The bus is 250 kbps — slow in CAN terms — so a Pi with SocketCAN ingests it without
dropping frames. A dedicated MCU capture front-end was considered and rejected as
over-engineering for this bandwidth.

## Data Rate & Capacity

These were the owner's headline concerns.

- **Ingest:** a loaded truck J1939 bus at 250 kbps runs ~30–50% utilization ≈
  **~1000–2000 frames/second** while the engine is running.
- **On-disk rate:** after MongoDB block compression (CAN IDs repeat heavily, so raw
  frames compress well), roughly **0.5–1.5 GB per driving day** at commercial hours
  (~10–11 h/day).
- **Capacity on a 2 TB NVMe:** **~3–10 years** of raw at those rates; a 4 TB drive
  doubles that. This is why keep-forever on-device is viable with no archive tier.

These figures are estimates. See *Open Items* — the real frame rate will be measured
on the actual truck to firm them up.

## Architecture

```
   Truck J1939 bus (CAN2, 250 kbps, 29-bit)
        │  CANH / CANL  (listen-only tap)
        ▼
   CAN FD HAT (MCP2518FD) ── SocketCAN can0
        │
        ▼
   Capture service  ──►  MongoDB (time-series, bucketed)  ──►  NVMe (2–4 TB)
        ▲                        ▲
        │                        │  bound to WiFi iface, LAN-restricted
   RTC (wall clock)         Laptop (Compass / mongosh) when parked on home WiFi
```

Three isolated units, each independently understandable and testable:

### 1. Capture service
- **Does:** reads every frame from SocketCAN `can0` and writes it to MongoDB.
- **Interface in:** SocketCAN `can0` (listen-only, 250 kbps), kernel/hardware
  receive timestamps (`SO_TIMESTAMP`) for precision, RTC for wall-clock base.
- **Interface out:** inserts frame documents into the MongoDB time-series collection.
- **Depends on:** SocketCAN up, RTC set, MongoDB reachable locally.
- Starts on boot (systemd), restarts on failure.

### 2. Storage (MongoDB time-series)
- **Does:** durably store raw frames in compact, compressed, queryable form.
- **Data model:** a **time-series collection** — `timeField` = frame timestamp,
  `metaField` = CAN ID (and/or PGN + source address). Bucketing collapses the
  tens of millions of frames per driving day from individual documents into
  compressed buckets, which is what makes raw-every-frame practical.
- **Frame record (logical shape):** `{ t, id (29-bit), dlc, data[<=8] }`.
- **Retention:** keep-forever (no TTL).
- **Optional metadata:** an ignition-derived trip/session id per frame for
  convenient per-drive queries (nice-to-have, not required for iteration 1).

### 3. Query access
- **Does:** let the owner query stored frames from a laptop.
- **Interface:** MongoDB bound to the WiFi interface, restricted to the home LAN
  (authentication enabled + host firewall). Query with **Compass or mongosh** —
  no bespoke software this iteration.
- Raw frames are returned as-is; turning them into named signals is the future
  decode tool.

## Power & Reliability

This is a first-class requirement, not a footnote — it is what separates "logs for
years" from "corrupt database in a month." Continuous logging in a truck faces:

- **Cranking dips:** 12 V rails sag to 6–9 V during engine start.
- **Load dumps / transients:** brief high-voltage spikes.
- **Abrupt key-off mid-write:** the corruption killer if unmanaged.

Mitigations:

1. **Wide-input automotive DC-DC (9–36 V)** rated for cranking dips and load-dump
   transients — not a naive buck converter.
2. **Ignition-sense → graceful shutdown:** an ignition-on signal wired to a Pi GPIO.
   On key-off, the Pi detects ignition-low, flushes and cleanly shuts down MongoDB,
   unmounts the NVMe, and powers down — riding the gap on the supercap/UPS.
3. **NVMe, never SD:** SD cards fail quickly under continuous write; NVMe has the
   endurance and survives power events better.
4. **MongoDB journaling** as the durability safety net beneath the clean-shutdown
   path.
5. **RTC** so timestamps are correct when there is no WiFi/NTP on the road.

## Open Items (validate during build)

1. **Measure the real bus frame rate on the truck** to firm up the 0.5–1.5 GB/day
   storage estimate — the Pi can count frames itself once it is on the bus.
2. **Select the specific UPS HAT** that bundles RTC + ignition-sense cleanly (avoids
   stacking separate HATs).
3. **Confirm the MCP2518FD HAT's listen-only configuration** under SocketCAN.

## Future Iterations (out of scope here, noted for continuity)

- **Decode tool:** raw frames → named J1939 signals using the local `j1939` MongoDB,
  likely built on the existing `tools/ovgt-telemetry` Node tooling. Decode-on-read
  keeps stored data lossless and lets decoders improve retroactively.
- Optional web UI, richer trip/session browsing, export helpers.
