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

### Actuator module — AFTER (C-Next `actuator.cnx` → generated `actuator.cpp`)

| Severity | Count | Notes |
|----------|-------|-------|
| HIGH | 0 | |
| MEDIUM | 0 | |
| LOW | 2 | 2× `badBitmaskCheck` (`\| (x << 0)` from C-Next bit-write codegen at offset 0) |

**Net:** the legacy dead PWM functions are gone (not carried into C-Next), and
the remaining findings are trivial/generated. More importantly, the C-Next
version is *structurally* safer than the C++ original: no raw pointers, MISRA
rules enforced at the language level, and the `Can0.write()` that previously ran
in a 20 ms timer **ISR** now runs in `Loop()` (main context).

## Conversion Progress

| File | Status | Notes |
|------|--------|-------|
| `src/display/actuator.cnx` | Done | FlexCAN_T4 RX (store-only) + `Loop()` sends latest vane-position setpoint in main context |

The vane position is a **latest-value singleton**, so the TX needs no queue — `Loop()`
(main context, ~10 ms) just sends `appData.actuatorDemandedPosition`. (An earlier draft
used a `CanTxQueue` SPSC ring buffer; removed — a FIFO is the right tool for the *future*
TCM→ECU CAN-message *forwarding* feature, not for a single setpoint.)

## Next Candidates

- `src/sensors/j1939.cpp` — RX decode already extracted to testable
  `j1939Decode` C++; the bus glue is a natural next C-Next module.
- `src/control/boostController.cpp` — small, self-contained.
- `src/domain/ovgt.cpp` — the orchestrator; convert last, once the leaf modules
  are C-Next.

## Firmware Size (teensy41, after actuator conversion)

```
FLASH: code:64876  data:12472  headers:8664   free for files:8040452
RAM1:  variables:34976  code:59688
RAM2:  variables:12416
```

Comparable to the hand-written version — C-Next transpiles to equivalent C++,
so there is no runtime/size penalty.

## Known transpiler issues found (filed upstream)

**Policy: we do NOT work around C-Next bugs — we fix them in c-next and contribute
back.** No shims, no "de-facto safe" hacks, no avoiding a language feature
downstream. Dependent features wait on the upstream fix.

- **c-next#998** — scope-level `atomic` variable definitions drop `volatile`
  (global `atomic` keeps it). Found while prototyping a scope-level SPSC ring
  buffer. The actuator does **not** rely on it (vane position is a singleton with
  no scope `atomic`), so nothing here is blocked. When the TCM→ECU forwarding FIFO
  is built (which needs scope `atomic`), **#998 must be fixed in c-next first** —
  do not ship a workaround.
- **c-next#997** — feature request: first-class ISR-safe queue (for the future
  CAN-message forwarding FIFO).

## Verification approach

The actuator is verified by (1) transpilation-output inspection and (2) on-target
build + bench testing — matching the reference project (OSSM), which has no native
unit tests of its C-Next code. The pure-C++ logic modules (`exhaustBrakeLogic`,
`j1939Decode`) remain covered by the native Unity suite. (Note: C-Next
`atomic`/`critical {}` code can't execute on x86 — PRIMASK/asm — and c-next's own
runner auto-skips x86 execution of such code; relevant once the forwarding FIFO
lands.)
