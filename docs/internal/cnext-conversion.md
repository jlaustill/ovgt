# C-Next Conversion Status

Tracks the incremental migration of OVGT firmware from hand-written C++ to
C-Next (transpiled). Modeled on OSSM's conversion log.

## Static Analysis Results (cppcheck via `pio check`)

### Actuator module — BEFORE (hand-written `actuator.cpp`)

| Location | Finding |
|----------|---------|
| `actuator.cpp:97` | `variableScope`: scope of `pwmValue` can be reduced |
| `actuator.cpp:79` | `unusedFunction`: `SetPWM` (legacy PWM path — dead) |
| `actuator.cpp:96` | `unusedFunction`: `CalibrateLoop` (legacy PWM path — dead) |

(`Initialize`/`Loop` also flagged "unused" only because cppcheck analyzed the
file in isolation — they are called from `ovgt.cpp`.)

### Actuator module — AFTER (C-Next `actuator.cnx` → generated `actuator.cpp` + `can_tx_queue.cpp`)

| Severity | Count | Notes |
|----------|-------|-------|
| HIGH | 0 | |
| MEDIUM | 0 | |
| LOW | 3 | 2× `badBitmaskCheck` (`\| (x << 0)` from C-Next bit-write codegen at offset 0); 1× `unreadVariable` `ignored` (intentional — drop-newest enqueue result) |

**Net:** the legacy dead PWM functions are gone (not carried into C-Next), and
the remaining findings are trivial/generated. More importantly, the C-Next
version is *structurally* safer than the C++ original: no raw pointers, MISRA
rules enforced at the language level, and `atomic`/`critical {}` ISR primitives
instead of an unguarded `Can0.write()` in a timer ISR.

## Conversion Progress

| File | Status | Notes |
|------|--------|-------|
| `src/display/can_tx_queue.cnx` | Done | SPSC ring buffer (`atomic` + `critical {}`) |
| `src/display/actuator.cnx` | Done | FlexCAN_T4 RX (store-only) + timer-enqueue/Loop-drain TX |

## Next Candidates

- `src/sensors/j1939.cpp` — RX decode already extracted to testable
  `j1939Decode` C++; the bus glue is a natural next C-Next module.
- `src/control/boostController.cpp` — small, self-contained.
- `src/domain/ovgt.cpp` — the orchestrator; convert last, once the leaf modules
  are C-Next.

## Firmware Size (teensy41, after actuator conversion)

```
FLASH: code:65068  data:12472  headers:8472   free for files:8040452
RAM1:  variables:34976  code:59880
RAM2:  variables:12416
```

Comparable to the hand-written version — C-Next transpiles to equivalent C++,
so there is no runtime/size penalty.

## Known transpiler issues hit during conversion (filed upstream)

- **c-next#998** — scope-level `atomic` variable definitions drop `volatile`
  (global `atomic` keeps it). Affects the `CanTxQueue` indices. De-facto safe
  on this single-core Cortex-M7 (function-call boundaries + the `critical {}`
  memory barrier), but the generated indices should be `volatile`; remove the
  workaround note once fixed.
- **c-next#997** — feature request: first-class ISR-safe queue (we hand-rolled
  the SPSC ring buffer from `atomic` + `critical {}`).

## Verification approach

C-Next code using `atomic`/`critical {}` cannot be executed on x86 (PRIMASK/asm),
and c-next's own test runner auto-skips execution of such code. So the queue and
actuator are verified by (1) transpilation-output inspection and (2) on-target
build + bench testing — matching the reference project (OSSM), which has no
native unit tests of its C-Next code. The pure-C++ logic modules
(`exhaustBrakeLogic`, `j1939Decode`) remain covered by the native Unity suite.
