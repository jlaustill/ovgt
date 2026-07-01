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

Built on a spare Raspberry Pi 5; NVMe drives are already on hand. The power path is
a discrete two-stage chain (see Power & Reliability) rather than a single power HAT.

| Part | Choice | Notes |
|------|--------|-------|
| Compute | **Raspberry Pi 5 (8 GB)** | Runs MongoDB (arm64), built-in WiFi. On hand. |
| CAN interface | **CAN FD HAT (MCP2518FD)** | Taps CANH/CANL, **listen-only**, 250 kbps, appears as SocketCAN `can0`. |
| Storage | **NVMe HAT + 2–4 TB NVMe** | On hand. Keep-forever headroom (see capacity). NVMe, never SD — write endurance. |
| Power — Stage 1 | **Mean Well wide-input DC-DC** (e.g. DDR-30 series, 9–36 V in, 5 V/6 A out) + input TVS | Converts dirty truck 12 V to clean, in-spec 5 V; wide input so a load dump lands inside range. Certified brick replaces a DIY surge-stopper. |
| Power — Stage 2 | **SCap UPS Board** (supercapacitor, 5 V-in/5 V-out pass-through + hold-up, power-loss GPIO) | Rides out key-off (15–110 s hold-up) so the Pi flushes + unmounts cleanly. Its power-loss pin is the shutdown trigger. |
| Time | Handled by the **RTC HAT** (or an RTC on the chosen stack) | Real wall-clock timestamps with no internet on the road. |

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
- **Load dumps / transients:** brief high-voltage spikes (a 12 V system can reach
  ~35 V+ on a load dump).
- **Abrupt key-off mid-write:** the corruption killer if unmanaged. A DC-DC
  converter alone cannot solve this — it is a pass-through with no energy reservoir,
  so input loss propagates to output in milliseconds. Un-flushed MongoDB writes and,
  worse, a consumer NVMe's in-flight flash-translation-layer update can corrupt on a
  hard cut ("won't mount," not just one bad record). This is why Stage 2 exists.

**Power path — discrete two stages** (chosen over a single power HAT because no HAT
delivers Pi 5's ~5 A *and* real automotive transient tolerance *and* seconds of
hold-up):

1. **Stage 1 — Mean Well wide-input DC-DC + input TVS.** Converts the dirty truck
   12 V to a clean, in-spec 5 V. A wide-input model (e.g. DDR-30, 9–36 V) keeps a
   load dump *inside* its input range instead of over-volting it; the TVS clamps the
   fast microsecond transients its input filter won't fully absorb. This shields the
   downstream Stage 2 board from all automotive nastiness.
2. **Stage 2 — SCap UPS Board (supercapacitor).** Fed the clean 5 V from Stage 1, it
   runs in 5 V-in/5 V-out pass-through while charging its supercaps. On key-off the
   Stage 1 output collapses; the SCap board detects the loss, asserts its power-loss
   GPIO, and rides the Pi down on the supercaps (15–110 s hold-up) — far more than
   the ~15–30 s needed. Supercaps (not Li-ion) mean no battery to degrade and a wide
   temperature tolerance.
3. **Graceful shutdown.** The SCap power-loss pin drives the Pi's built-in
   `gpio-shutdown` device-tree overlay: GPIO low → clean Linux shutdown → MongoDB
   flush → NVMe unmount, all completed within the supercap hold-up window. No custom
   ignition-sense circuit required; wiring to switched (ignition) 12 V makes key-off
   the shutdown trigger directly.
4. **NVMe, never SD:** SD cards fail quickly under continuous write; NVMe has the
   endurance and, behind the hold-up, is unmounted cleanly.
5. **MongoDB journaling** as the durability safety net beneath the clean-shutdown
   path.
6. **RTC** so timestamps are correct when there is no WiFi/NTP on the road.
7. **Cab install** (behind a panel / under a seat — not the dash or engine bay) keeps
   ambient within the supercaps' comfortable range, making the module effectively
   maintenance-free for years.

## Open Items (validate during build)

1. **Measure the real bus frame rate on the truck** to firm up the 0.5–1.5 GB/day
   storage estimate — the Pi can count frames itself once it is on the bus.
2. **Confirm the exact Mean Well model** — verify 5 V/≥6 A at 12 V input, wide-input
   range covers the truck's worst-case load dump, and physical mounting.
3. **Verify the SCap UPS Board** — that it truly passes 5 V through (not double-
   convert) under load, its continuous current/thermal behavior, and its supercap
   temperature rating against the cab install location.
4. **Confirm the MCP2518FD HAT's listen-only configuration** under SocketCAN, and
   that it coexists on the 40-pin header with any RTC/other GPIO users.

## Future Iterations (out of scope here, noted for continuity)

- **Decode tool:** raw frames → named J1939 signals using the local `j1939` MongoDB,
  likely built on the existing `tools/ovgt-telemetry` Node tooling. Decode-on-read
  keeps stored data lossless and lets decoders improve retroactively.
- Optional web UI, richer trip/session browsing, export helpers.
