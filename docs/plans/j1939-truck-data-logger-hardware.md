# J1939 Truck Data Logger — Design

**Date:** 2026-07-01 · **Hardware direction revised:** 2026-07-03
**Status:** Draft. **Compute platform under active reconsideration** — the original
Raspberry Pi 5 build (below) is superseded by a rugged in-vehicle PC direction; see
**[Hardware Selection (2026-07-03 re-evaluation)](#hardware-selection-2026-07-03-re-evaluation)**.
Decision is intentionally **paused** for the owner to reflect before committing.
**Scope:** Iteration 1 — raw frame capture, storage, and query. Decode-to-named-signals is explicitly a later iteration.

> **Note on this document.** The software architecture (SocketCAN capture → MongoDB
> time-series → LAN query) and the requirements/data-rate analysis are **platform-agnostic
> and still current** regardless of the compute box chosen. Only the *compute + power
> hardware* changed. The original Pi 5 hardware/power sections are retained below,
> marked **superseded**, as a documented DIY fallback.

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

## Hardware Selection (2026-07-03 re-evaluation)

### Update (2026-07-04): compute requirement shrank — K300 is now overkill
Once the plan settled on the **OCT box as the CAN front-end**, the ruggedness requirement moved
*off* the computer and *onto* the OCT. The OCT is already in a **waterproof Deutsch PCB case with
a DTM connector**, so it mounts under the dash looking semi-OEM and owns the automotive/marine-grade,
permanently-installed, dog-proof part — which was the **K300's entire justification**. So the K300
(and the whole rugged in-vehicle-PC tier) is now **overkill**. The remaining need is just a **small
always-on computer to replace the laptop**, running the SocketCAN→MongoDB capture stack — that's
*storage + compute*, not ruggedness. A small fanless mini-PC / SBC with NVMe is plenty; no
ignition-grade in-vehicle PC is needed, because the OCT front-end is the rugged/isolated piece.
**This is exactly why parking the purchase was right — the requirement materially changed.** Revisit
box selection later against this shrunken spec.

### Why we revisited
Pricing out the original Raspberry Pi 5 build — Pi 5 + Mean Well PSU + NVMe HAT + SCap
UPS HAT + a designed/3D-printed case with heat management — lands at **~$500+**. At that
price, with the Pi's known thermal issues and the owner's history of Pi boards dying,
the question flipped from *"cheapest build"* to **"if I'm spending real money, buy the
right tool once."** (See memory `feedback_buy_it_for_life_threshold`: ~$500+ purchases
are longevity/ruggedness decisions, not cost-minimization.)

### The corrected requirement
This must be a **general-purpose computer that runs the owner's own open-source Linux
stack** (SocketCAN + the MongoDB capture service above) — analogy: *"the best hardware
to install OpenWRT on, not a router."* Top priority: **NVMe (non-volatile), never an
SD/removable card.** Plus: automotive wide-voltage power with graceful ignition-off
shutdown, and a CAN interface for the 250 kbps J1939 bus.

### Categorically rejected: turnkey proprietary CAN-logger appliances
Purpose-built loggers (CSS Electronics **CANedge1/2/3**, **Influx Rebel LT/CT**, **HEM
Data DAWN**) are excellent at what they do and were researched in detail, but they are
**disqualified on two axes**: they run *vendor firmware* (not the owner's stack) and log
to *removable SD/SDXC* (not NVMe). Wrong category — the "buy a router" answer to an
"install OpenWRT" question.

### Chosen direction: OnLogic Karbon K300 (rugged in-vehicle PC)
A fanless rugged in-vehicle computer, treated by the owner as the **"north star" box**:

- **Onboard M.2 2280 NVMe** (non-volatile ✓, cheap large SSDs).
- **Onboard classical CAN 2.0B** — no USB dongle to snap off. Classical 2.0B is the
  *correct* spec: the J1939 truck bus is 250 kbps classical; CAN-FD would be wasted.
- **9–36 V input + built-in "Intelligent Ignition Sensing"** → graceful delayed
  shutdown on key-off, protecting the NVMe filesystem. **No external DC-DC or UPS HAT
  needed** — it collapses the Pi's two-stage power chain into one sealed unit.
- **Explicit ISO-7637-2 / ISO-16750 automotive-transient immunity** — a genuine
  differentiator; no competitor cites it (they use E-Mark or rail-spec EN50155).
- **First-class Ubuntu**, pre-imaged; US-based (Vermont) support; transparent pricing;
  1-yr warranty extendable to 5-yr.
- CPU is a fanless industrial **Apollo Lake Atom** — ample for headless 24/7 logging.
- Base **~$1,640** (approximate; live configurator/quote un-fetchable).

**Key reveal:** OnLogic's rugged line is *"Rugged by Neousys"* — OnLogic **resells Neousys
hardware**. So the K300 is a Neousys VTC cousin plus OnLogic's US support / validated
Ubuntu image / warranty. The premium buys **support & integration, not unique silicon**;
Neousys-direct (via CoastIPC/Assured) is the same DNA for less, minus that support layer.

### Competitive landscape — does anything beat the K300 in its class?
Researched ~15 rivals (Neousys, Cincoze, Vecow, Sintrones, Lanner, Advantech, Premio).
**No box matches the K300 on all four vehicle-critical axes at once** (onboard NVMe +
onboard CAN + *built-in* ignition + ISO-7637-2). Closest alternatives each force a trade:

- **Sintrones VBOX-3132** — only true feature-for-feature match, actually *beats* K300
  (2× M.2 NVMe, newer Amston Lake CPU) — but **quote-only**, no ISO-7637-2, 2-yr warranty.
- **Neousys Nuvo-2610VTC / POC-451VTC** — cheaper (~$825–975), newer CPU, strong Linux
  docs — but **M.2 is SATA-only (no NVMe)** + bolt-on CAN module.
- **Cincoze DI-1200** — verified public **$1,499**, **12th-gen Core** (much stronger CPU/GPU),
  Ubuntu 22.04 — but NVMe unconfirmed, CAN + ignition are **add-on modules**, no ISO-7637-2.
- **Lanner V3S** — closest on *onboard* CAN + ignition + E-Mark — but **no NVMe** (SATA) and
  it's a larger NVR-form-factor box.

**Regret-from-a-newer-box risk is LOW:** industrial in-vehicle PCs run **5–7-year
lifecycles** on long-availability Intel embedded SKUs. You will not be obsoleted out of
availability in months (the anti-Pi). The K300's only real soft spots are that it sits on
the **oldest silicon** in the field (weakest GPU) and has **a single CAN channel**.

### The real decision axis: how real is the dash-HMI + HVAC future?
A future vision surfaced (owner *thinking out loud, not committed*): the same box also
driving a **dash touchscreen HMI** — live J1939 gauges + historical graphs from the
onboard log ("truck data warehouse") — and doubling as **HVAC controls**. That vision
pokes the K300's exact two weaknesses: weak GPU (sluggish heavy graphing) and one CAN
channel (HVAC implies a second body bus). So the box choice hinges on this probability:

| Honest read on HMI/HVAC future | Buy | ~Price | Rationale |
|---|---|---|---|
| **Someday-maybe (<~30%)** | **OnLogic Karbon K300** | ~$1,640 | Best-in-class sealed rugged logger; add/repurpose later — industrial gear holds value, so not trapped. |
| **Feels real (>~50%), value** | **Cincoze DI-1200** + CAN + ignition modules | ~$1,500–1,650 | 12th-gen Core GPU for a fluid HMI + **dual CAN-FD** (engine + HVAC) at ~K300 money. Trade: modules, no ISO-7637-2. |
| **Feels real, no-compromise** | **Neousys Nuvo-9100VTC** | ~$2,200–3,200 | 13th-gen, UHD 770 GPU, **native dual isolated CAN** + ignition, in-vehicle certified. Full-vision box. |
| **Stay in OnLogic ecosystem for HMI** | **OnLogic Karbon 800** (Iris Xe) | ~$1,900–2,700 | Best GPU of the group, dual 4K DisplayPort; still 1 CAN (add a card for HVAC). |

**The display is a red herring for regret** — it's a plain HDMI/DP + USB-touch peripheral
(wide-voltage automotive touch monitor, e.g. Faytech 8–36 V ~$300–500), fully decoupled
and **deferrable indefinitely**. The hard-to-reverse decisions all live *inside the box*:
**CPU/GPU tier and CAN-channel count**. Over-provision those once; pick the screen later.
"Buy once, cry once" logic: in the 30–50% middle, *step up* — buying twice is the exact
outcome being avoided, and the delta to cover the future is small against the total.

> **Footnote (2026-07-03, to process later):** the owner's instinct is that under
> *single-device / single-concern*, the **dash HMI + HVAC should be its own device**, not
> folded into the logger. If so, the logger never needs the strong GPU or the second CAN
> channel — which **collapses the tier decision back to the plain K300 (or the cheap
> N100/DIY path)** and removes all "step up" pressure above. Open design seam for that
> future: how the HMI box gets data — its own listen-only CAN tap, or reading the logger's
> MongoDB over the in-cab LAN. Not today's problem; noted so it isn't lost.

### Phase 0 (now): bootstrap the logging stack on the existing Ubuntu laptop — $0
**Decision (2026-07-03):** don't wait on — or spend on — hardware. The owner already has an
Ubuntu laptop that travels with the truck **and** surplus **OCT boxes** (ordered a min-of-5,
several spare). Build the full software stack **now**, and defer dedicated-hardware purchase
until the stack is proven, funds are set aside, and the HMI/HVAC question is resolved.

**Hardware: repurpose a surplus OCT box as the USB-CAN interface.**
- OCT (`~/code/oct`) is a **Teensy 4.1 MicroMod** with **onboard dual CAN transceivers**
  (J1939 = CAN2, Cummins = CAN3, both 250 kbps via FlexCAN_T4), J1850 VPW, native USB, and
  12–18 V input. It already runs on the truck's J1939 bus and talks over USB serial — the
  hard hardware bring-up is proven. Better physical USB-CAN hardware than a bought dongle.
- **NOT the candleLight path.** candleLight / `gs_usb` is STM32-only firmware and will not run
  on a Teensy (the owner's candleLight memory conflated it with the slcan bridge below). The
  Teensy path is an **slcan bridge**: firmware reads FlexCAN frames → emits slcan ASCII over USB
  serial → on the laptop `slcand -o -c -s5 /dev/ttyACM0 slcan0` + `ip link set slcan0 up` →
  SocketCAN `slcan0`. Feeds the **same SocketCAN → MongoDB capture service** as any future box.
- **PRIOR ART EXISTS AND IS BUILT** — the sketch is already written:
  `~/code/kuminz-re/firmware/CM848_S90140.06_analysis/tools/teensy-slcan/src/main.cpp` (compiled
  `firmware.hex` present). Complete slcan command set, FlexCAN on **CAN2**, default 250k (S5).
  Phase 0 is largely "flash it → `slcand` → point the capture service at `slcan0`."
- **Fallback:** a bought USB-CAN dongle with in-kernel SocketCAN — **Innomaker USB2CAN** (~$35,
  isolated, `gs_usb`, classical CAN) or CANable 2.0 / PEAK — is the drop-in alternative.

**Three must-square items before trusting it as a passive logger:**
1. **Add listen-only (LOM) — the existing sketch does NOT do this.** It runs FlexCAN in normal
   mode (supports `t`/`T` TX and, critically, **ACKs every frame it receives**). Extra ACKing
   isn't catastrophic (CAN nodes ACK by design), but the doc rule is "never TX *or ACK*," so
   enable FlexCAN **listen-only** before opening the channel. Verify whether FlexCAN_T4 exposes
   it or set `CTRL1[LOM]` directly. **This is the safety gate.**
2. **Bring-up uses `slcand`, not the `can()` bashrc helper** (that helper is for the native
   CANable v1.0 — `ip link … type can bitrate` — which does NOT apply to a serial slcan device).
3. **Confirm J1939 = CAN2 on the OCT** — OCT's own files disagree (CLAUDE.md: J1939=CAN2;
   `oct-domain.cpp` print: J1939=CAN3). Verify via `candump slcan0` (expect 29-bit extended
   frames); switch the firmware to CAN3 if silent.

**Physical:** tap the truck J1939 CANH/CANL (confirm which FlexCAN maps to it — README vs
oct-domain print statements disagree on CAN2/CAN3↔J1939); tuck the laptop under the seat,
protected as best as possible (dog included).

**Why this is the right first step:** truly zero cost (surplus hardware + owned laptop), proves
the SocketCAN → MongoDB stack end-to-end, and **measures the real bus frame rate** (Open Item #1)
— the number that actually sizes any future dedicated box. Use the tool before buying one.

### If instead a bare-SBC DIY path is ever chosen (cheaper tier, researched for completeness)
- **x86:** an industrial **N100 fanless box with a verified 9–36 V terminal input + M.2
  2280 NVMe** (CWWK/Topton/Partaker-class) — x86 = bulletproof mainline SocketCAN. ~$180–260.
- **ARM:** **FriendlyElec NanoPC-T6** (native 12 V + full M.2 2280 PCIe x4) or Radxa Rock 5B.
- **Cross-cutting finding — do NOT trust on-chip CAN on hobby SBCs.** RK3588 on-chip CAN-FD
  has mainline support only for the **RK3568** (kernel ≥6.12); on RK3588 it needs vendor-kernel
  overlays. Use a **USB-CAN adapter** for a portable, dependable `can0` regardless of board.
- Accessories that make any bare SBC truck-ready: **Mini-Box DCDC-USB-200** (~$93, wide input +
  ignition-sensed graceful shutdown, open-source Linux driver) and a **USB-CAN adapter** —
  **Innomaker USB2CAN** (~$35, isolated, in-kernel `gs_usb`, classical CAN = right for 250k
  J1939) or **PEAK PCAN-USB isolated** (~$220, gold-standard mainline `peak_usb`).

### Verify before purchase
1. **K300 onboard CAN — native SocketCAN (`can0`) or an internal serial/ACM gateway?**
   Research saw it "exposed as a serial/ACM device," which would mean `slcand`, not a native
   controller. This underwrites the whole "no dongle" appeal — confirm directly with OnLogic.
2. **Confirm the CAN option is included** in the specific K300 config/SKU (it can be optional).
3. **Get real quotes** — every configured price above is behind a JS configurator or quote wall.

### Key sources
OnLogic Karbon K300 / K700 / K800 (onlogic.com/store); Neousys in-vehicle Nuvo-VTC /
POC-VTC (neousys-tech.com, CoastIPC); Cincoze DI-1200 (cincoze.com, mitxpc.com);
Sintrones VBOX-3132; Lanner V3S (lannerinc.com); Mini-Box DCDC-USB-200 (mini-box.com,
github.com/mini-box/dcdc-usb); CANable/Innomaker/PEAK USB-CAN. Full session research
(60 verified claims + competitor matrices) captured in conversation; prices approximate
2026 USD, verify before budgeting.

---

## Hardware (SUPERSEDED — original Raspberry Pi 5 DIY design, retained as fallback)

> Superseded 2026-07-03 by the rugged in-vehicle PC direction above. Retained because the
> reasoning (especially Power & Reliability) documents the failure modes any chosen box must
> survive, and this remains a valid lower-cost DIY fallback if the rugged-PC path is dropped.

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

## Power & Reliability (SUPERSEDED — Pi 5 two-stage power path; failure-mode analysis still applies)

> Part of the superseded Pi 5 design. A rugged in-vehicle PC (e.g. K300) solves this in
> one sealed unit via built-in ignition sensing + ISO-7637-2 immunity, replacing the
> Mean Well + SCap two-stage chain below. The **failure modes** described here (cranking
> dips, load dumps, abrupt key-off mid-write) are exactly what any chosen box must survive,
> so this analysis remains the reliability rationale regardless of platform.

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
