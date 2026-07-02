# systemHealth.cnx — Reset-Cause & Watchdog (Phase 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every Teensy reset self-documenting in MongoDB and add on-chip watchdog auto-recovery, by routing already-present-but-unlogged signals into the telemetry stream — firmware-only, strictly additive.

**Architecture:** Two new C-Next modules following the repo's existing pure-logic/hardware split (`boostBprLogic` vs `boostController`). `systemHealthLogic.cnx` is a **pure** module (no hardware includes → transpiles to C mode → native-testable) holding the reset-cause bit-decode, loop-timing accumulator, and Vin conversion/min-tracker. `systemHealth.cnx` is a **hardware** module (includes `<Arduino.h>`, uses the C-Next `register` construct) that reads SRC/WDOG registers, drives the watchdog, samples Vin, and delegates all math to the logic module. `ovgt.cpp` calls the hardware module and adds eight fields to the existing 10 Hz `"t"` telemetry record.

**Tech Stack:** C-Next (`cnext` binary), Teensy 4.1 / IMXRT1062, PlatformIO (`teensy41` + `native` envs), Unity tests.

## Global Constraints

- **No runtime config** — the watchdog enable and Vin pin/divider are compile-time `#define`s only. Never add serial/TUI config mutation (driving hazard).
- **No control/tuning changes** — this work is purely additive telemetry + watchdog. Do not touch PID, boost, exhaust-brake, or actuator logic.
- **Fix C-Next upstream, never work around** — if the `cnext` binary rejects valid syntax or mis-transpiles (especially the first-use `register` construct), file a minimal repro and fix it in the c-next repo; do not hand-edit generated files or add a C/C++ shim to dodge it.
- **Generated `.c`/`.h`/`.cpp`/`.hpp` are committed** — regeneration is deterministic. Native tests build from committed generated files (the `native` env has no `cnext` pre-script), so every C-Next edit must be followed by a `cnext` transpile and the generated files committed alongside the source.
- **Spell names out** — no invented abbreviations. Universal ones (RPM, CAN, ADC, PWM, JSON) are fine.
- **Build/test commands:** firmware `pio run -e teensy41`; logic tests `pio test -e native` (filter `-f test_system_health`); transpile `cnext <entry.cnx> --header-out include`.
- **IMXRT bases/offsets (verified in Teensy core `imxrt.h`):** `SRC` base `0x400F8000` (SRSR @ `0x08`, GPR9 @ `0x40`); `WDOG1` base `0x400B8000` (WCR @ `0x00`, WSR @ `0x02`). `SRC_GPR5` is reserved by the core — do not use it.

---

## SRC_SRSR reset-cause bit map (verified in `imxrt.h`)

| Bit | `imxrt.h` name | Meaning | Decoded code | String |
| --- | -------------- | ------- | ------------ | ------ |
| 8 | `SRC_SRSR_TEMPSENSE_RST_B` | on-die temp-sensor trip | 6 | `tempsense` |
| 7 | `SRC_SRSR_WDOG3_RST_B` | watchdog 3 | 5 | `wdog3` |
| 6 | `SRC_SRSR_JTAG_SW_RST` | JTAG software reset | 7 | `jtag` |
| 5 | `SRC_SRSR_JTAG_RST_B` | JTAG reset | 7 | `jtag` |
| 4 | `SRC_SRSR_WDOG_RST_B` | watchdog 1 | 4 | `wdog` |
| 3 | `SRC_SRSR_IPP_USER_RESET_B` | external reset pin | 3 | `pin` |
| 2 | `SRC_SRSR_CSU_RESET_B` | CSU | 8 | `csu` |
| 1 | `SRC_SRSR_LOCKUP_SYSRESETREQ` | CPU lockup **or** software reset (a crash/fault reset lands here) | 2 | `lockup` |
| 0 | `SRC_SRSR_IPP_RESET_B` | power-on reset (**a brownout returns here too — no dedicated brownout bit**) | 1 | `por` |
| — | none set | — | 0 | `unknown` |

**Decode precedence** (highest first, because a reset can set several bits): `wdog(4) > wdog3(7) > tempsense(8) > lockup(1) > pin(3) > jtag(5/6) > csu(2) > por(0) > unknown`. Brownout-vs-key-on POR is distinguished by *context* (`pg` / `vin_mv`), not the register.

---

## Task 1: Pure-logic module scaffold + reset-cause decode (TDD)

**Files:**
- Create: `src/domain/systemHealthLogic.cnx`
- Create: `test/test_system_health/test_system_health.cpp`
- Modify: `cnext_build.py` (add entry)
- Modify: `platformio.ini` (`[env:native]` `build_src_filter`)
- Generated (commit): `src/domain/systemHealthLogic.c`, `include/systemHealthLogic.h`

**Interfaces:**
- Produces: `uint8_t SystemHealthLogic_decodeResetCause(uint32_t srsr)` → code per the bit-map table (0=unknown,1=por,2=lockup,3=pin,4=wdog,5=wdog3,6=tempsense,7=jtag,8=csu).

- [ ] **Step 1: Add the build entry for the new module**

In `cnext_build.py`, change the `entries` list to include the logic module:

```python
    entries = [
        Path("src/display/actuator.cnx"),
        Path("src/domain/json.cnx"),
        Path("src/domain/systemHealthLogic.cnx"),
    ]
```

In `platformio.ini`, add the generated `.c` to the native filter (append to the existing `+<...>` list):

```ini
build_src_filter = -<*> +<control/exhaustBrakeLogic.cpp> +<control/boostBprLogic.cpp> +<sensors/j1939Decode.cpp> +<sensors/j1939Encode.cpp> +<sensors/compressorEfficiency.cpp> +<sensors/cotSettle.cpp> +<domain/json.c> +<domain/systemHealthLogic.c>
```

- [ ] **Step 2: Write the failing test**

Create `test/test_system_health/test_system_health.cpp`:

```cpp
#include <unity.h>
#include "systemHealthLogic.h"

void setUp(void) {}
void tearDown(void) {}

// One SRSR bit per known reset cause -> decoded code.
void test_decode_each_cause(void) {
    TEST_ASSERT_EQUAL_UINT8(1, SystemHealthLogic_decodeResetCause(1u << 0)); // por
    TEST_ASSERT_EQUAL_UINT8(2, SystemHealthLogic_decodeResetCause(1u << 1)); // lockup
    TEST_ASSERT_EQUAL_UINT8(3, SystemHealthLogic_decodeResetCause(1u << 3)); // pin
    TEST_ASSERT_EQUAL_UINT8(4, SystemHealthLogic_decodeResetCause(1u << 4)); // wdog
    TEST_ASSERT_EQUAL_UINT8(5, SystemHealthLogic_decodeResetCause(1u << 7)); // wdog3
    TEST_ASSERT_EQUAL_UINT8(6, SystemHealthLogic_decodeResetCause(1u << 8)); // tempsense
    TEST_ASSERT_EQUAL_UINT8(7, SystemHealthLogic_decodeResetCause(1u << 5)); // jtag
    TEST_ASSERT_EQUAL_UINT8(8, SystemHealthLogic_decodeResetCause(1u << 2)); // csu
    TEST_ASSERT_EQUAL_UINT8(0, SystemHealthLogic_decodeResetCause(0u));      // unknown
}

// Watchdog wins when both watchdog and POR bits are set (real resets set several).
void test_decode_precedence_wdog_over_por(void) {
    uint32_t srsr = (1u << 4) | (1u << 0);
    TEST_ASSERT_EQUAL_UINT8(4, SystemHealthLogic_decodeResetCause(srsr));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_decode_each_cause);
    RUN_TEST(test_decode_precedence_wdog_over_por);
    return UNITY_END();
}
```

- [ ] **Step 3: Create the module and transpile (expect the test to fail to build — no header yet)**

Create `src/domain/systemHealthLogic.cnx` (PURE — no `#include`, so it transpiles to C mode like `json.cnx`):

```cnx
// Pure health-telemetry math: reset-cause decode, loop-timing, Vin conversion.
// No hardware access here so it transpiles to C mode and is native-testable.
// The hardware module systemHealth.cnx feeds real register/ADC values in.
scope SystemHealthLogic {

    // SRC_SRSR bit -> stable numeric code (see plan bit-map table).
    public u8 decodeResetCause(u32 srsr) {
        if (srsr[4] = true) { return 4; }   // WDOG_RST_B      -> wdog
        if (srsr[7] = true) { return 5; }   // WDOG3_RST_B     -> wdog3
        if (srsr[8] = true) { return 6; }   // TEMPSENSE_RST_B -> tempsense
        if (srsr[1] = true) { return 2; }   // LOCKUP_SYSRESETREQ -> lockup
        if (srsr[3] = true) { return 3; }   // IPP_USER_RESET_B -> pin
        if (srsr[6] = true) { return 7; }   // JTAG_SW_RST     -> jtag
        if (srsr[5] = true) { return 7; }   // JTAG_RST_B      -> jtag
        if (srsr[2] = true) { return 8; }   // CSU_RESET_B     -> csu
        if (srsr[0] = true) { return 1; }   // IPP_RESET_B     -> por
        return 0;                            // unknown
    }
}
```

Transpile:

```bash
cnext src/domain/systemHealthLogic.cnx --header-out include
```

Expected: creates `src/domain/systemHealthLogic.c` and `include/systemHealthLogic.h`.

- [ ] **Step 4: Run the test to verify it now passes**

Run: `pio test -e native -f test_system_health`
Expected: `test_decode_each_cause` PASS, `test_decode_precedence_wdog_over_por` PASS.

- [ ] **Step 5: Commit**

```bash
git add src/domain/systemHealthLogic.cnx src/domain/systemHealthLogic.c include/systemHealthLogic.h \
        test/test_system_health/test_system_health.cpp cnext_build.py platformio.ini
git commit -m "feat(health): reset-cause SRSR decode (C-Next pure logic, native-tested)"
```

---

## Task 2: Loop-timing accumulator (TDD)

**Files:**
- Modify: `src/domain/systemHealthLogic.cnx`
- Modify: `test/test_system_health/test_system_health.cpp`
- Regenerate + commit: `src/domain/systemHealthLogic.c`, `include/systemHealthLogic.h`

**Interfaces:**
- Consumes: (none new)
- Produces:
  - `void SystemHealthLogic_loopTimingReset(void)`
  - `void SystemHealthLogic_loopTimingRecord(uint32_t intervalMicros)`
  - `uint32_t SystemHealthLogic_loopTimingMax(void)` — max interval since last reset
  - `uint32_t SystemHealthLogic_loopTimingAvg(void)` — mean interval since last reset (0 if none)

- [ ] **Step 1: Write the failing test**

Append to `test/test_system_health/test_system_health.cpp` (add the two functions and their `RUN_TEST` lines in `main`):

```cpp
void test_loop_timing_max_and_avg(void) {
    SystemHealthLogic_loopTimingReset();
    SystemHealthLogic_loopTimingRecord(10000);
    SystemHealthLogic_loopTimingRecord(12000);
    SystemHealthLogic_loopTimingRecord(11000);
    TEST_ASSERT_EQUAL_UINT32(12000, SystemHealthLogic_loopTimingMax());
    TEST_ASSERT_EQUAL_UINT32(11000, SystemHealthLogic_loopTimingAvg()); // (10000+12000+11000)/3
}

void test_loop_timing_reset_clears(void) {
    SystemHealthLogic_loopTimingReset();
    SystemHealthLogic_loopTimingRecord(50000);
    SystemHealthLogic_loopTimingReset();
    TEST_ASSERT_EQUAL_UINT32(0, SystemHealthLogic_loopTimingMax());
    TEST_ASSERT_EQUAL_UINT32(0, SystemHealthLogic_loopTimingAvg());
}
```

Add to `main`:

```cpp
    RUN_TEST(test_loop_timing_max_and_avg);
    RUN_TEST(test_loop_timing_reset_clears);
```

- [ ] **Step 2: Run to verify it fails to build (functions undefined)**

Run: `pio test -e native -f test_system_health`
Expected: FAIL / build error — `SystemHealthLogic_loopTimingReset` undefined.

- [ ] **Step 3: Implement in the module, then transpile**

Add these fields and methods inside `scope SystemHealthLogic` in `src/domain/systemHealthLogic.cnx`:

```cnx
    u32 loopMax <- 0;
    u32 loopSum <- 0;
    u32 loopCount <- 0;

    public void loopTimingReset() {
        this.loopMax <- 0;
        this.loopSum <- 0;
        this.loopCount <- 0;
    }

    public void loopTimingRecord(u32 intervalMicros) {
        if (intervalMicros > this.loopMax) {
            this.loopMax <- intervalMicros;
        }
        this.loopSum <- this.loopSum + intervalMicros;
        this.loopCount <- this.loopCount + 1;
    }

    public u32 loopTimingMax() {
        return this.loopMax;
    }

    public u32 loopTimingAvg() {
        if (this.loopCount = 0) {
            return 0;
        }
        return this.loopSum / this.loopCount;
    }
```

Transpile: `cnext src/domain/systemHealthLogic.cnx --header-out include`

- [ ] **Step 4: Run to verify pass**

Run: `pio test -e native -f test_system_health`
Expected: all four tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/domain/systemHealthLogic.cnx src/domain/systemHealthLogic.c include/systemHealthLogic.h test/test_system_health/test_system_health.cpp
git commit -m "feat(health): loop-timing max/avg accumulator (native-tested)"
```

---

## Task 3: Vin conversion + minimum tracker (TDD)

**Files:**
- Modify: `src/domain/systemHealthLogic.cnx`
- Modify: `test/test_system_health/test_system_health.cpp`
- Regenerate + commit: `src/domain/systemHealthLogic.c`, `include/systemHealthLogic.h`

**Interfaces:**
- Produces:
  - `uint32_t SystemHealthLogic_rawToMillivolts(uint32_t rawAdc)` — 10-bit ADC (0..1023), 3.3 V ref, ÷ divider ratio, returns supply mV. Ratio is fixed here: divider passes 1/6 of Vin (e.g. 100 kΩ / 20 kΩ), so `mV = rawAdc * 3300 / 1023 * 6`. (`u32` param avoids `analogRead` int→u16 narrowing at the call site.)
  - `void SystemHealthLogic_vinReset(void)`
  - `void SystemHealthLogic_vinRecord(uint32_t rawAdc)`
  - `int32_t SystemHealthLogic_vinMinMillivolts(void)` — minimum since reset; `-1` if no samples recorded (the "no divider fitted" sentinel).

- [ ] **Step 1: Write the failing test**

Append to the test file (and add `RUN_TEST` lines):

```cpp
void test_vin_raw_to_millivolts(void) {
    // Half-scale (raw 512) through a 1/6 divider: 512*3300/1023*6 ~= 9911 mV.
    TEST_ASSERT_UINT32_WITHIN(15, 9911, SystemHealthLogic_rawToMillivolts(512));
    TEST_ASSERT_EQUAL_UINT32(0, SystemHealthLogic_rawToMillivolts(0));
}

void test_vin_min_tracks_and_sentinel(void) {
    SystemHealthLogic_vinReset();
    TEST_ASSERT_EQUAL_INT32(-1, SystemHealthLogic_vinMinMillivolts()); // no samples yet
    SystemHealthLogic_vinRecord(600); // ~11.6 V
    SystemHealthLogic_vinRecord(500); // ~9.7 V  (the dip)
    SystemHealthLogic_vinRecord(620);
    int32_t mv = SystemHealthLogic_vinMinMillivolts();
    TEST_ASSERT_INT32_WITHIN(40, 9677, mv); // min corresponds to raw 500
}
```

Add to `main`:

```cpp
    RUN_TEST(test_vin_raw_to_millivolts);
    RUN_TEST(test_vin_min_tracks_and_sentinel);
```

- [ ] **Step 2: Run to verify it fails to build**

Run: `pio test -e native -f test_system_health`
Expected: FAIL — `SystemHealthLogic_rawToMillivolts` undefined.

- [ ] **Step 3: Implement, then transpile**

Add inside `scope SystemHealthLogic`:

```cnx
    // Vin divider ratio: the resistor network passes 1/DIVIDER_RATIO of Vin to
    // the ADC pin. 100k/20k -> ratio 6. ADC is 10-bit (0..1023) at 3.3 V ref.
    const u32 VIN_DIVIDER_RATIO <- 6;

    bool vinSeen <- false;
    u32 vinMin <- 0;

    public u32 rawToMillivolts(u32 rawAdc) {
        u32 atPin <- rawAdc * 3300 / 1023;
        return atPin * this.VIN_DIVIDER_RATIO;
    }

    public void vinReset() {
        this.vinSeen <- false;
        this.vinMin <- 0;
    }

    public void vinRecord(u32 rawAdc) {
        u32 mv <- this.rawToMillivolts(rawAdc);
        bool first <- this.vinSeen = false;
        if (first = true) {
            this.vinMin <- mv;
            this.vinSeen <- true;
        } else {
            if (mv < this.vinMin) {
                this.vinMin <- mv;
            }
        }
    }

    public i32 vinMinMillivolts() {
        if (this.vinSeen = false) {
            return -1;
        }
        return this.vinMin[0, 32];
    }
```

Transpile: `cnext src/domain/systemHealthLogic.cnx --header-out include`

- [ ] **Step 4: Run to verify pass**

Run: `pio test -e native -f test_system_health`
Expected: all six tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/domain/systemHealthLogic.cnx src/domain/systemHealthLogic.c include/systemHealthLogic.h test/test_system_health/test_system_health.cpp
git commit -m "feat(health): Vin raw->mV conversion + min tracker with no-divider sentinel"
```

---

## Task 4: Hardware module `systemHealth.cnx` (registers, watchdog, boot counter, Vin sample)

Hardware bring-up — verified on-device in Task 6, not natively unit-tested. **This is the first use of the C-Next `register` construct in this repo; Step 1 de-risks it before building the whole module.**

**Files:**
- Create: `include/SystemHealthConfig.h`
- Create: `src/domain/systemHealth.cnx`
- Modify: `cnext_build.py` (add entry)
- Generated (commit): `src/domain/systemHealth.cpp`, `include/systemHealth.hpp`

**Interfaces:**
- Consumes: all `SystemHealthLogic_*` functions from Tasks 1–3.
- Produces (called by `ovgt.cpp` in Task 5):
  - `void SystemHealth_init(void)` — read+clear SRSR, snapshot cause code, increment `SRC_GPR9` boot counter, enable WDOG1 if enabled.
  - `void SystemHealth_feed(void)` — service WDOG1 + record inter-feed interval.
  - `void SystemHealth_sampleSupply(void)` — `analogRead` Vin, feed the min-tracker.
  - `uint8_t SystemHealth_resetCauseCode(void)`
  - `uint32_t SystemHealth_bootCount(void)`
  - `uint32_t SystemHealth_loopMicrosMax(void)` / `uint32_t SystemHealth_loopMicrosAvg(void)`
  - `int32_t SystemHealth_supplyMillivolts(void)`
  - `void SystemHealth_windowReset(void)` — reset per-emit accumulators (loop timing + Vin min); called by `ovgt.cpp` right after emitting a `"t"` record.

- [ ] **Step 1: Create the config header**

Create `include/SystemHealthConfig.h` (use `static const`, not `#define`, to match
the proven `VaneConfig.h` ↔ `actuator.cnx` pattern for reading C constants from
C-Next):

```c
#ifndef SystemHealthConfig_h
#define SystemHealthConfig_h

#include <stdint.h>

// Compile-time only (driving hazard forbids runtime config).

// 1 = enable the IMXRT WDOG1 hardware watchdog; 0 = disabled (fast escape hatch
// if it ever false-trips into a boot loop). Dead code is eliminated when 0.
static const uint8_t OVGT_WATCHDOG_ENABLED = 1;

// Watchdog timeout via WDOG1_WCR.WT: timeout = (WT + 1) * 0.5 s. 3 -> 2.0 s.
// Conservative start; tighten from logged loop_us_max data.
static const uint8_t OVGT_WATCHDOG_WT = 3;

// 1 = a Vin resistor divider is fitted on OVGT_VIN_ADC_PIN; 0 = not fitted
// (vin_mv reports -1). Firmware ships with 0 until the divider is added.
static const uint8_t OVGT_VIN_ENABLED = 0;

// Spare analog-capable pin for the Vin divider (owner selects a free pin;
// pin 40 = A16 on Teensy 4.1). Unused while OVGT_VIN_ENABLED = 0.
static const uint8_t OVGT_VIN_ADC_PIN = 40;

#endif
```

- [ ] **Step 2: Smoke-test the two first-use C-Next features before building the real module**

Two things are used here for the first time in this repo: the `register` construct
(ADR-004) and a **C-Next scope calling another included C-Next scope** (this module
will call `SystemHealthLogic`). Validate both with one throwaway file
`src/domain/_regsmoke.cnx`:

```cnx
#include "systemHealthLogic.h"

register SRC @ 0x400F8000 {
    SRSR: u32 rw @ 0x08,
    GPR9: u32 rw @ 0x40,
}

scope RegSmoke {
    public u32 readReg() {
        return SRC.SRSR;                              // register read/deref
    }
    public u8 crossScope(u32 srsr) {
        return SystemHealthLogic.decodeResetCause(srsr);  // C-Next -> C-Next call
    }
}
```

Run: `cnext src/domain/_regsmoke.cnx --header-out include`
Expected: transpiles cleanly — `SRC.SRSR` dereferences `0x400F8008`, and
`crossScope` emits a call to `SystemHealthLogic_decodeResetCause`.

**If the `register` construct fails:** STOP. File a minimal C-Next bug (this file
is the repro) and fix `register`/bit-index support upstream per the Global
Constraints. Do not add a shim.

**If the cross-scope call fails:** prefer fixing it upstream. As a
schedule-preserving fallback, restructure so `ovgt.cpp` (C++) does the
orchestration — `systemHealth.cnx` exposes raw accessors (e.g.
`readAndClearResetStatus() -> u32`, `readVinRaw() -> u32`) and `ovgt.cpp` calls
both `SystemHealth_*` and `SystemHealthLogic_*` directly. That C++→C-Next path is
already proven (`ovgt.cpp` calls `Json_*`), so it is a safe fallback that keeps the
logic in C-Next.

Once both pass, delete the smoke files:

```bash
rm -f src/domain/_regsmoke.cnx src/domain/_regsmoke.c include/_regsmoke.h
```

- [ ] **Step 3: Write the hardware module**

Create `src/domain/systemHealth.cnx`:

```cnx
#include <Arduino.h>
#include "SystemHealthConfig.h"
#include "systemHealthLogic.h"

// IMXRT1062 System Reset Controller: reset status + retained scratch registers.
register SRC @ 0x400F8000 {
    SRSR: u32 rw @ 0x08,   // reset status; write-1-to-clear
    GPR9: u32 rw @ 0x40,   // retained across warm reset, zeroed on POR (GPR5 is core-reserved)
}

// IMXRT1062 Watchdog 1 (16-bit registers).
register WDOG1 @ 0x400B8000 {
    WCR: u16 rw @ 0x00,    // control
    WSR: u16 rw @ 0x02,    // service (feed): 0x5555 then 0xAAAA
}

scope SystemHealth {
    u8 causeCode <- 0;
    u32 bootCounter <- 0;
    u32 lastFeedMicros <- 0;
    bool feedSeen <- false;

    void enableWatchdog() {
        u16 wcr <- 0x0030;              // reset default: SRS + WDA held high
        wcr[2] <- true;                 // WDE: enable watchdog
        wcr[1] <- true;                 // WDBG: suspend while halted under debugger
        wcr[8, 8] <- OVGT_WATCHDOG_WT;  // WT: (WT+1)*0.5 s timeout
        WDOG1.WCR <- wcr;               // single write (WDE/WT are write-once)
    }

    public void init() {
        u32 srsr <- SRC.SRSR;
        this.causeCode <- SystemHealthLogic.decodeResetCause(srsr);
        SRC.SRSR <- srsr;               // write-1-to-clear the latched bits

        u32 boots <- SRC.GPR9 + 1;      // retained boot counter
        SRC.GPR9 <- boots;
        this.bootCounter <- boots;

        SystemHealthLogic.loopTimingReset();
        SystemHealthLogic.vinReset();
        this.feedSeen <- false;

        if (OVGT_WATCHDOG_ENABLED = 1) {
            this.enableWatchdog();
        }
    }

    public void feed() {
        u32 now <- micros();
        bool first <- this.feedSeen = false;
        if (first = true) {
            this.feedSeen <- true;
        } else {
            u32 interval <- now - this.lastFeedMicros;   // u32 wrap is correct for micros()
            SystemHealthLogic.loopTimingRecord(interval);
        }
        this.lastFeedMicros <- now;

        if (OVGT_WATCHDOG_ENABLED = 1) {
            WDOG1.WSR <- 0x5555;
            WDOG1.WSR <- 0xAAAA;
        }
    }

    public void sampleSupply() {
        if (OVGT_VIN_ENABLED = 1) {
            u32 raw <- analogRead(OVGT_VIN_ADC_PIN);   // analogRead is 0..1023
            SystemHealthLogic.vinRecord(raw);
        }
    }

    public void windowReset() {
        SystemHealthLogic.loopTimingReset();
        SystemHealthLogic.vinReset();
    }

    public u8 resetCauseCode() { return this.causeCode; }
    public u32 bootCount() { return this.bootCounter; }
    public u32 loopMicrosMax() { return SystemHealthLogic.loopTimingMax(); }
    public u32 loopMicrosAvg() { return SystemHealthLogic.loopTimingAvg(); }
    public i32 supplyMillivolts() { return SystemHealthLogic.vinMinMillivolts(); }
}
```

Add the entry to `cnext_build.py` (after the logic module so its header exists):

```python
    entries = [
        Path("src/display/actuator.cnx"),
        Path("src/domain/json.cnx"),
        Path("src/domain/systemHealthLogic.cnx"),
        Path("src/domain/systemHealth.cnx"),
    ]
```

Transpile: `cnext src/domain/systemHealth.cnx --header-out include`
Expected: creates `src/domain/systemHealth.cpp` and `include/systemHealth.hpp`.

- [ ] **Step 4: Build the firmware to verify it compiles (no hardware behavior yet)**

Run: `pio run -e teensy41`
Expected: SUCCESS. `systemHealth.cpp` compiles; `SystemHealth_*` symbols link.

- [ ] **Step 5: Commit**

```bash
git add include/SystemHealthConfig.h src/domain/systemHealth.cnx src/domain/systemHealth.cpp include/systemHealth.hpp cnext_build.py
git commit -m "feat(health): systemHealth.cnx hardware module (SRSR, WDOG1, boot counter, Vin)"
```

---

## Task 5: Wire into `ovgt.cpp` — telemetry fields + watchdog feed

**Files:**
- Modify: `src/domain/ovgt.cpp` (includes; `setup()`; `loop()`; `handleDebug()`)

**Interfaces:**
- Consumes: all `SystemHealth_*` functions from Task 4.
- The `reset_cause`, `boot_count`, `crash`, `setup_ms` values are session-constant and emitted on every `"t"` record; `pg`, `vin_mv`, `loop_us_max`, `loop_us_avg` are per-window.

- [ ] **Step 1: Add includes and a reset-cause-code → string helper**

At the top of `src/domain/ovgt.cpp`, after the existing includes, add:

```cpp
#include <systemHealth.hpp>

static const char *resetCauseName(uint8_t code) {
    switch (code) {
        case 1: return "por";
        case 2: return "lockup";
        case 3: return "pin";
        case 4: return "wdog";
        case 5: return "wdog3";
        case 6: return "tempsense";
        case 7: return "jtag";
        case 8: return "csu";
        default: return "unknown";
    }
}
```

Add two file-scope statics near `volatile uint32_t ovgt::count;`:

```cpp
static bool bootCrashPresent = false;
static uint32_t bootSetupMillis = 0;
```

- [ ] **Step 2: Initialize SystemHealth first in `setup()` and capture crash + setup time**

In `ovgt::setup()`, make `SystemHealth_init()` the first call after `Serial.begin(115200);` (it must clear SRSR and start the watchdog before the blocking sensor inits). Capture the crash flag from the existing `CrashReport` block, and measure total setup duration. The edited `setup()` head and tail:

Replace the opening of `setup()` (from `Serial.begin` through the `CrashReport` block) with:

```cpp
    Serial.begin(115200);

    elapsedMillis setupTimer;
    SystemHealth_init();

    // Capture whether a hard-fault crash report survived the reset (C++ core
    // object, so read here) before printing clears it. The bool rides telemetry;
    // the full text stays on serial until Phase 2 routes it into Mongo.
    bootCrashPresent = (bool)CrashReport;
    if (CrashReport) {
        Serial.println("=== CRASH REPORT ===");
        Serial.print(CrashReport);
        Serial.println("=== END CRASH REPORT ===");
    }
```

At the very end of `setup()`, replace `Serial.println("Setup complete");` with:

```cpp
    bootSetupMillis = setupTimer;
    Serial.println("Setup complete");
```

Add `SystemHealth_feed();` calls after each blocking `Initialize()` in `setup()` so a slow init cannot trip the 2 s watchdog during boot. Insert `SystemHealth_feed();` after each of: `AdcSensors::Initialize();`, `TitSensor::Initialize();`, `CotSensor::Initialize();`, `Fram::Initialize();`, `BoostController::Initialize();`, `ExhaustBrakeController::Initialize();`, `Actuator_Initialize();`, `J1939::Initialize();`.

- [ ] **Step 3: Feed the watchdog and sample supply every 100 Hz tick**

In `ovgt::loop()`, immediately after `loopElapsed = 0;` and `count++;`, add:

```cpp
    SystemHealth_feed();
    SystemHealth_sampleSupply();
```

- [ ] **Step 4: Add the eight health fields to the telemetry record**

In `handleDebug()`, immediately before `Json_addBool("brake", appData.exhaustBrakeActive);`, add:

```cpp
    Json_addStr("reset_cause", resetCauseName(SystemHealth_resetCauseCode()));
    Json_addUint("boot_count", SystemHealth_bootCount());
    Json_addBool("crash", bootCrashPresent);
    Json_addBool("pg", appData.pgFault);
    Json_addInt("vin_mv", SystemHealth_supplyMillivolts());
    Json_addUint("loop_us_max", SystemHealth_loopMicrosMax());
    Json_addUint("loop_us_avg", SystemHealth_loopMicrosAvg());
    Json_addUint("setup_ms", bootSetupMillis);
```

Then, immediately after `Serial.write('\n');` at the end of `handleDebug()`, reset the per-window accumulators so `loop_us_*` and `vin_mv` describe each 100 ms window independently:

```cpp
    SystemHealth_windowReset();
```

- [ ] **Step 5: Confirm the telemetry record still fits `string<640>`**

The Json buffer is `string<640>` and `test_json`'s capacity test proves the old worst case fits with margin. Extend that guard for the new fields. In `test/test_json/test_json.cpp`, inside `test_capacity_no_overflow`, before `Json_end();`, add:

```cpp
    Json_addStr("reset_cause", "tempsense");
    Json_addUint("boot_count", 4294967295u);
    Json_addBool("crash", true);
    Json_addBool("pg", true);
    Json_addInt("vin_mv", 60000);
    Json_addUint("loop_us_max", 4294967295u);
    Json_addUint("loop_us_avg", 4294967295u);
    Json_addUint("setup_ms", 4294967295u);
```

Run: `pio test -e native -f test_json`
Expected: `test_capacity_no_overflow` PASS (`Json_overflowed()` false, `Json_len() < 640`). If it now overflows, raise the `string<N>` size in `json.cnx`, re-transpile `json.cnx`, and commit the regenerated `json.c`/`json.h`.

- [ ] **Step 6: Build the firmware**

Run: `pio run -e teensy41`
Expected: SUCCESS.

- [ ] **Step 7: Commit**

```bash
git add src/domain/ovgt.cpp test/test_json/test_json.cpp
# include json.c/json.h too if the buffer was resized in Step 5
git commit -m "feat(health): emit reset-cause/boot/pg/vin/loop-timing + feed watchdog in ovgt loop"
```

---

## Task 6: On-hardware verification

No code — a device checklist proving the resets are now self-documenting. Uses the serial-capture procedure that does not reset the board (`stty -F /dev/ttyACM0 115200 raw -echo` then `timeout <s> cat /dev/ttyACM0`) and MongoDB queries against the live session.

- [ ] **Step 1: Flash and confirm a clean boot record**

```bash
pio run -e teensy41 -t upload
```
Then capture serial and confirm each `"t"` line now carries `reset_cause`, `boot_count`, `crash:false`, `pg`, `vin_mv:-1` (no divider yet), `loop_us_max`, `loop_us_avg`, `setup_ms`. On a normal power-up flash, `reset_cause` should read `por`.

- [ ] **Step 2: Confirm loop timing is sane at idle**

At idle, `loop_us_avg` should be ≈ 10000 (the 100 Hz tick) and `loop_us_max` within a few ms of it. A wildly larger value means the feed placement in `loop()` is wrong — investigate before trusting the watchdog.

- [ ] **Step 3: Prove the watchdog fires and is decoded**

Temporarily add an infinite `while(1){}` behind a compile guard in `loop()` (or a `delay(5000)`), flash, and confirm: the board resets after ~2 s, and the **next** boot's `reset_cause` reads `wdog`. Remove the test hang and reflash.

- [ ] **Step 4: Prove crash capture**

Temporarily force a null dereference in a test build, flash, let it fault and reset, and confirm the next boot emits `crash:true` and `reset_cause:lockup` (fault-induced resets latch `LOCKUP_SYSRESETREQ`). Remove and reflash.

- [ ] **Step 5: Confirm boot counter increments across warm resets**

Trigger two watchdog resets in a row (Step 3 method) and confirm `boot_count` increases by one each warm reset, then verify a full power-cycle returns `boot_count` to 1 (POR zeroes `SRC_GPR9`) — this power-vs-warm distinction is itself a diagnostic.

- [ ] **Step 6: Verify it lands in MongoDB**

With the telemetry tool running, confirm the new fields are queryable, e.g. distinct reset causes seen this session:

```
db.telemetry.aggregate([
  {$match: {sessionId: "<current>"}},
  {$group: {_id: "$reset_cause", n: {$sum: 1}, maxLoopUs: {$max: "$loop_us_max"}}}
])
```
Expected: reset causes present, and `maxLoopUs` gives the first real data point for later right-sizing the watchdog timeout.

- [ ] **Step 7: Drive and collect**

Take a normal drive. Afterward, if resets occurred, read `reset_cause` for each boot in Mongo — this is the deliverable: electrical (`por` with a `pg`/`vin_mv` dip) vs. firmware (`wdog`/`lockup`) is now answered from data. Record findings for the Phase 2 spec.

---

## Notes for the implementer

- **C-Next equality vs assignment:** `<-` is assignment, `=` is equality. `if (x = 5)` is a comparison. Compound: `x +<- 1`.
- **Why two modules:** `systemHealthLogic.cnx` has no `#include`, so `cnext` emits C-mode `.c`/`.h` that the `native` env compiles and Unity tests directly. `systemHealth.cnx` includes `<Arduino.h>` and uses `register`, so it emits C++-mode `.cpp`/`.hpp` and only builds under `teensy41`.
- **Transpile then commit:** the `native` env has no `cnext` pre-script; it builds committed generated files. After every `.cnx` edit, run the `cnext ... --header-out include` command shown and commit the regenerated files with the source.
- **Do not touch `SRC_GPR5`** — the Teensy core uses it for its own crash breadcrumb; the boot counter uses `SRC_GPR9`.
