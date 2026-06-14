# Actuator C-Next Pilot — Design

## Summary

Convert the actuator module (`src/display/actuator.cpp/.h`) to C-Next
(`actuator.cnx`) as the first step of migrating OVGT to C-Next for MISRA-grade
standards. The conversion also fixes the leading suspect for the controller's
reboot loop: the CAN bus write currently performed inside a 20 ms `IntervalTimer`
ISR. The new design moves the write out of interrupt context using an
`atomic`/`critical` single-producer/single-consumer (SPSC) ring buffer — the
timer ISR enqueues, the main loop dequeues and transmits.

This pilots the full C-Next toolchain on OVGT: build integration, C++↔C-Next
interop, FlexCAN_T4 usage from C-Next, and ISR-safe concurrency primitives.

## Rationale & References

- **Reboot is most likely ISR/concurrency, not a memory bug.** `pio check`
  (cppcheck + clang-tidy, full framework context) found zero fault-class issues
  in our code — only a benign `%u`/signed-int format nit (`ovgt.cpp:56`). That
  points away from memory bugs and toward the one architectural red flag:
  `Actuator::handleCanTimer()` calling `Can0.write()` from a timer ISR while two
  CAN RX callbacks also run in ISR context, all touching shared state unguarded.
  (Reboot cause is not yet *confirmed* — `CrashReport` on the truck is the
  decisive check. This conversion removes the highest-probability suspect.)
- **Working reference: OSSM `src/Display/J1939Bus.cnx`.** OSSM is a Teensy +
  `tonton81/FlexCAN_T4` + `jlaustill/J1939` project whose CAN bus is written in
  C-Next. It proves C-Next uses FlexCAN_T4 **natively** — no C++ shim:
  `#include "FlexCAN_T4.h"`, `FlexCAN_T4<CAN1,...> canBus;`, `canBus.write()/.begin()/.onReceive(...)`,
  and an ISR callback written in C-Next. We mirror its structure.
- **No first-class queue.** ADR-104 (ISR-safe queues) is still **Research**
  (open design questions, no grammar keyword, no stdlib type, no `.cnx` usage —
  the only "queue" in the repo is a FreeRTOS C-interop fixture). Per the
  "only Implemented/Accepted ADRs are working syntax" rule, we build the ring
  buffer ourselves from `atomic` (ADR-049) + `critical {}` (ADR-050), exactly
  as OSSM buffers its inbound config command.
- **Build integration reference: ogauge `cnext_build.py`** — a `pre:` PlatformIO
  script that runs `cnext`, commits generated output, and headers land in
  `include/`.

## Current behavior to preserve (from `actuator.cpp`)

- **Bus:** CAN3 @ 500 kbps, FIFO + FIFO interrupt, `onReceive(canSniff)`.
- **RX** (`canSniff`, id `0x4EB`): decode `rawPosition` (bytes 2-3),
  `motorLoad` (6-7), `status` (0), `temp` (5); compute
  `actuatorReportedPosition = 100 - map(rawPosition, 0,1000, 0,100)`.
- **TX** (`SendCanPosition`, id `0x4EA`): `buf[0] = map(100 - position, 0,100, 0,250)`,
  `Can0.write(msg)`.
- **The bug:** a 20 ms `IntervalTimer` (`handleCanTimer`) calls
  `SendCanPosition(appData.actuatorDemandedPosition)` — i.e. `Can0.write()` from
  ISR context.
- `ACTUATOR_MODE_CAN = true`; the PWM/analogWrite path is legacy/unused on this
  build and will **not** be carried into the pilot (YAGNI — can return later).
- Calibration constants: PWM 100..247 (legacy), CAN cmd 0..250, CAN raw 0..1000.

## Architecture: `src/display/actuator.cnx`

A `scope Actuator` mirroring `J1939Bus.cnx`.

### CAN instance & shared state
```
#include <Arduino.h>
#include "FlexCAN_T4.h"
#include "AppData.h"            // existing C++ struct; C-Next reads/writes appData

scope Actuator {
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can0;
    ...
}
```

### TX ring buffer (SPSC, atomic + critical)
The timer ISR is the producer; `Loop()` (main context) is the consumer. The
buffer holds the demanded vane position byte (the only thing the timer produces);
the full `CAN_message_t` is built in the consumer just before `can0.write()`.

```
const u8 TX_CAPACITY <- 8;                 // power of two; small is plenty at 20ms
u8[8] txPositions;
atomic u8 txHead <- 0;                      // producer index
atomic u8 txTail <- 0;                      // consumer index

private bool txEnqueue(u8 position) {        // called from timer ISR
    u8 next <- (txHead + 1) & (TX_CAPACITY - 1);
    bool full <- (next = txTail);
    if (full = true) { return false; }       // drop-newest if full (stale anyway)
    txPositions[txHead] <- position;
    txHead <- next;
    return true;
}

private bool txDequeue(u8 outPosition) {     // called from Loop()
    bool empty <- (txHead = txTail);
    if (empty = true) { return false; }
    critical {
        outPosition <- txPositions[txTail];
        txTail <- (txTail + 1) & (TX_CAPACITY - 1);
    }
    return true;
}
```
- Indices are `atomic` so producer/consumer see consistent values; `critical {}`
  guards the consumer's read+advance against the ISR. Capacity is a power of two
  so the wrap is a mask, not a modulo (MISRA-friendly, no division).
- "Drop-newest when full" is correct here: a missed position sample is
  immediately superseded by the next 20 ms sample.

### Functions (exported to C++ as `Actuator_*`)
- `void Initialize()` — `can0.begin()`, `setBaudRate(500000)`, `setMaxMB(16)`,
  `enableFIFO()`, `enableFIFOInterrupt()`, `onReceive(receiveCallbackISR)`,
  `mailboxStatus()`, and start the 20 ms timer whose handler calls
  `txEnqueue(appData.actuatorDemandedPosition)`.
- `void Loop()` — drain the TX buffer: while `txDequeue(pos)` succeeds, build the
  `CAN_message_t` (id `0x4EA`, `buf[0] = map(100 - pos, 0,100, 0,250)`) and
  `can0.write(msg)`. **All writes happen here, in main context.**
- `private void receiveCallbackISR(const CAN_message_t msg)` — id `0x4EB` only;
  store the decoded fields into `appData`. No CAN write, no processing in the ISR.
  (RX is store-only, matching the safe half of the OSSM pattern.)

### Interop & orchestration
- `ovgt.cpp` currently calls `Actuator::Initialize(pin)` and `Actuator::Loop()`.
  These become `Actuator_Initialize()` / `Actuator_Loop()` (C-Next underscore
  exports). The `pin` argument is dropped (CAN mode doesn't use it).
- The 20 ms `IntervalTimer` moves into `actuator.cnx` (it already lived in the
  actuator module).
- `appData` (C++ struct, `include/AppData.h`) is read/written directly from
  C-Next via the include — same global, no migration of AppData in this pilot.

## Build Integration

- Add `cnext_build.py` (adapted from ogauge) as a `pre:` script in
  `platformio.ini`. For this pilot it transpiles `src/display/actuator.cnx`
  (ovgt's entry is `main.cpp`, not `.cnx`, so we transpile the `.cnx` file
  directly rather than "follow includes from main").
- Generated `actuator.cpp` + header replace the handwritten ones; **generated
  files are committed** (per project rule). Remove the old handwritten
  `actuator.cpp`/`actuator.h`.
- Headers emitted to `include/` (matching ogauge); `build_src_filter` / include
  paths updated so the generated unit compiles and `ovgt.cpp` finds the header.

## Testing

- **Ring buffer logic** is the high-value, pure unit to test: enqueue/dequeue,
  empty, full (drop-newest), wrap-around, and producer/consumer interleaving.
  Written as a C-Next test (`actuator.test.cnx` → committed `.test.c/.test.h`),
  per the C-Next testing convention.
- **Static-analysis tracking:** record `pio check` (cppcheck + clang-tidy)
  results before and after, mirroring OSSM's `docs/internal/c-next-conversion.md`,
  to document the standards improvement.
- **On-hardware:** confirm vanes still command/track in manual mode, and (next
  truck session) confirm the reboot loop stops and `CrashReport` stays clean.

## Risks & Assumptions

- **Reboot cause unconfirmed.** This removes the top suspect (ISR CAN write); if
  reboots persist, the cause is elsewhere (power/brownout per the deployment
  environment, or RX-side shared `appData` races) — pursue with `CrashReport`.
- **C-Next reading the C++ `appData` struct** is assumed to work via include
  (all output compiles as C++). Validate early; if it doesn't, fall back to a
  small accessor or migrate the actuator's slice of state into C-Next.
- **FlexCAN_T4 templated instance + `onReceive` callback in C-Next** — proven by
  OSSM `J1939Bus.cnx`, so low risk.
- **Transpiler quirks** (e.g. ogauge's #982 const-array-param header fix) may
  surface; carry the same workarounds if needed.

## Out of Scope (this pilot)

- Converting other modules (`j1939`, `boostController`, `exhaustBrake*`,
  `adcSensors`, `ovgt`) — sequenced after the pilot proves the toolchain.
- The legacy PWM actuator path.
- Migrating `AppData` itself to C-Next.
- Root-causing a non-ISR reboot source (power/brownout) — separate track.
