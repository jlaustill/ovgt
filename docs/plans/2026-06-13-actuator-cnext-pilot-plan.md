# Actuator C-Next Pilot — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert the actuator module to C-Next (`actuator.cnx`), moving the CAN write out of the timer ISR via an `atomic`/`critical` SPSC ring buffer, as the first module of OVGT's C-Next migration.

**Architecture:** A `scope Actuator` mirroring OSSM's `J1939Bus.cnx` — `FlexCAN_T4<CAN3,…>` used natively (no shim). A 20 ms `IntervalTimer` ISR enqueues the demanded vane position into a ring buffer; `Loop()` drains it and performs `canBus.write()` in main context. RX callback is store-only. C++ `ovgt.cpp` calls the generated `Actuator_*()` functions; the C++ `appData` struct is accessed directly from C-Next.

**Tech Stack:** C-Next (`cnext` v0.2.16), Teensy 4.1 (PlatformIO), `tonton81/FlexCAN_T4`, Arduino `IntervalTimer`. References: OSSM `~/code/ossm/src/Display/J1939Bus.cnx` (FlexCAN+C-Next), ogauge `~/code/ogauge/cnext_build.py` (build), `~/.claude/cnext-ai-reference.md` (syntax).

---

## File Structure

- **Create** `src/display/can_tx_queue.cnx` — standalone, hardware-free SPSC ring buffer (`scope CanTxQueue`). Kept separate so it has zero hardware includes and is unit-testable in isolation.
- **Create** `src/display/actuator.cnx` — the C-Next actuator module (CAN init, RX store-only callback, timer producer, `Loop()` consumer). Includes `can_tx_queue.cnx`.
- **Create** `cnext_build.py` — PlatformIO `pre:` hook that transpiles `actuator.cnx` (adapted from ogauge; `cnext` follows the include into `can_tx_queue.cnx`).
- **Generated (committed)** `src/display/actuator.cpp`, `src/display/can_tx_queue.cpp` + `include/display/Actuator.hpp`, `include/display/CanTxQueue.hpp` — transpiler output. The old hand-written `actuator.cpp`/`actuator.h` are removed.
- **Create** `src/display/can_tx_queue.test.cnx` + committed `can_tx_queue.test.c/.h` — ring-buffer unit test (no hardware deps).
- **Modify** `platformio.ini` — add `extra_scripts` + include path.
- **Modify** `src/domain/ovgt.cpp` — call `Actuator_Initialize()` / `Actuator_Loop()`.
- **Create** `docs/internal/cnext-conversion.md` — static-analysis before/after (OSSM pattern).

**Interop facts (verified against OSSM):** `scope Actuator { void Initialize() {} }` → `Actuator_Initialize()` and `include/display/Actuator.hpp`. C++ includes `<display/Actuator.hpp>` and calls `Actuator_Initialize()`.

---

## Task 1: Build integration + skeleton scope (validate the toolchain & C++ interop)

This proves transpile → compile → link → call-from-C++ before any logic. Highest-risk unknowns (build wiring, `Actuator_*` interop) are validated first.

**Files:**
- Create: `cnext_build.py`, `src/display/actuator.cnx`
- Modify: `platformio.ini`, `src/domain/ovgt.cpp`
- Delete: `src/display/actuator.h`, `src/display/actuator.cpp` (old hand-written)

- [ ] **Step 1: Add the transpile hook** — create `cnext_build.py` (adapted from ogauge; transpiles the actuator entry directly since ovgt's entry is `main.cpp`):

```python
Import("env")
import subprocess, sys
from pathlib import Path

def transpile_cnext():
    entry = Path("src/display/actuator.cnx")
    if not entry.exists():
        return
    print("Transpiling actuator.cnx...")
    try:
        subprocess.run(
            ["cnext", str(entry), "--header-out", "include"],
            check=True, capture_output=True, text=True,
        )
    except subprocess.CalledProcessError as e:
        print("  ✗ C-Next transpilation failed")
        print(e.stderr)
        sys.exit(1)

transpile_cnext()
```

- [ ] **Step 2: Register the hook + include path** in `platformio.ini` under `[env:teensy41]` (add these two lines; leave the rest unchanged):

```ini
extra_scripts = pre:cnext_build.py
build_flags = -I include
```

- [ ] **Step 3: Write the skeleton `src/display/actuator.cnx`:**

```cnx
#include <Arduino.h>

scope Actuator {
    void Initialize() {
        global.Serial.println("Actuator (C-Next) initialized");
    }

    void Loop() {
    }
}
```

- [ ] **Step 4: Transpile it manually to verify output + header location:**

Run: `cnext src/display/actuator.cnx --header-out include`
Expected: SUCCESS; creates `src/display/actuator.cpp` and `include/display/Actuator.hpp` (verify with `ls include/display/Actuator.hpp src/display/actuator.cpp`).

- [ ] **Step 5: Remove the old hand-written actuator + switch callers.** Delete the old files and update `ovgt.cpp`.

```bash
git rm src/display/actuator.h src/display/actuator.cpp
```

In `src/domain/ovgt.cpp`, change the include `#include "display/actuator.h"` to:

```cpp
#include <display/Actuator.hpp>
```

Change `Actuator::Initialize();` (in `setup()`) to:

```cpp
    Actuator_Initialize();
```

Change `Actuator::Loop();` (the timed block in `loop()`, currently wrapped in the DWT cycle counters) to:

```cpp
    Actuator_Loop();
```

(Keep the surrounding `t0 = ARM_DWT_CYCCNT; … cyclesActuator += …` lines; only the call changes.)

- [ ] **Step 6: Build for the target:**

Run: `pio run -e teensy41`
Expected: SUCCESS. The pre-script transpiles `actuator.cnx`, the generated `actuator.cpp` compiles, and `ovgt.cpp` links against `Actuator_Initialize`/`Actuator_Loop`.

> If `cnext` emits `.h` instead of `.hpp`, or a different header path, adjust the `#include` in `ovgt.cpp` and the `--header-out` accordingly — match whatever the transpiler actually produces (confirm against ogauge's generated `include/` layout).

- [ ] **Step 7: Commit:**

```bash
git add cnext_build.py platformio.ini src/display/actuator.cnx src/display/actuator.cpp include/display/Actuator.hpp src/domain/ovgt.cpp
git commit -m "Add C-Next build hook and actuator skeleton scope

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 2: CAN bring-up + store-only RX callback (validate FlexCAN + appData interop)

Mirrors OSSM `J1939Bus.cnx` init/RX. Validates the two remaining interop unknowns: the `FlexCAN_T4<CAN3>` instance in C-Next, and reading/writing the C++ `appData` struct.

**Files:**
- Modify: `src/display/actuator.cnx`

- [ ] **Step 1: Add includes, the CAN instance, and the RX callback** to `actuator.cnx`. Replace the file body with:

```cnx
#include <Arduino.h>
#include "FlexCAN_T4.h"
#include "AppData.h"

scope Actuator {
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> canBus;

    const u32 CMD_ID <- 0x4EA;        // TX: position command
    const u32 FEEDBACK_ID <- 0x4EB;   // RX: actuator feedback

    // RX runs in ISR context: decode fields into appData ONLY (no write, no work).
    private void receiveCallbackISR(const CAN_message_t msg) {
        if (msg.id != FEEDBACK_ID) {
            return;
        }

        u16 rawPosition;
        rawPosition[8, 8] <- msg.buf[2];     // high byte -> bits 8..15
        rawPosition[0, 8] <- msg.buf[3];     // low byte  -> bits 0..7

        u16 motorLoad;
        motorLoad[8, 8] <- msg.buf[6];
        motorLoad[0, 8] <- msg.buf[7];

        // actuatorReportedPosition = 100 - (rawPosition * 100 / 1000)
        u32 raw <- rawPosition;
        u32 mapped <- raw * 100 / 1000;
        u32 reported <- 100 - mapped;

        global.appData.actuatorRawPosition <- rawPosition;
        global.appData.actuatorMotorLoad <- motorLoad;
        global.appData.actuatorStatus <- msg.buf[0];
        global.appData.actuatorTemp <- msg.buf[5];
        global.appData.actuatorReportedPosition <- reported[0, 8];
    }

    void Initialize() {
        this.canBus.begin();
        this.canBus.setBaudRate(500 * 1000);
        this.canBus.setMaxMB(16);
        this.canBus.enableFIFO();
        this.canBus.enableFIFOInterrupt();
        this.canBus.onReceive(this.receiveCallbackISR);
        this.canBus.mailboxStatus();
        global.Serial.println("Actuator (C-Next) initialized");
    }

    void Loop() {
    }
}
```

- [ ] **Step 2: Transpile and build:**

Run: `pio run -e teensy41`
Expected: SUCCESS.

> This is the interop checkpoint. If C-Next cannot resolve `global.appData.<field>` (because it doesn't parse `AppData.h` for field types), fall back to a tiny C++ accessor header (`app_data_access.h`) exposing `extern "C"` setters/getters and call those via `global.` — the same shape as a C boundary layer. Validate before proceeding; do not paper over a transpile error.

- [ ] **Step 3: Commit:**

```bash
git add src/display/actuator.cnx src/display/actuator.cpp include/display/Actuator.hpp
git commit -m "Add FlexCAN bring-up and store-only RX callback in actuator.cnx

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 3: TX ring buffer as a standalone scope (atomic + critical) — TDD

A dependency-free `scope CanTxQueue` in its own file — pure logic, no hardware includes, so it unit-tests in isolation. Power-of-two capacity (mask wrap, no division). Producer = timer ISR; consumer = `Loop()`.

**Files:**
- Create: `src/display/can_tx_queue.cnx`
- Create: `src/display/can_tx_queue.test.cnx`

- [ ] **Step 1: Write the failing test** `src/display/can_tx_queue.test.cnx`:

```cnx
#include "can_tx_queue.cnx"

void test_enqueue_then_dequeue() {
    CanTxQueue.reset();
    bool ok <- CanTxQueue.enqueue(42);
    assert(ok = true);
    u8 out <- 0;
    bool got <- CanTxQueue.dequeue(out);
    assert(got = true);
    assert(out = 42);
}

void test_empty_returns_false() {
    CanTxQueue.reset();
    u8 out <- 0;
    bool got <- CanTxQueue.dequeue(out);
    assert(got = false);
}

void test_full_drops_newest() {
    CanTxQueue.reset();
    // capacity 8 -> 7 usable slots before full
    for (u8 i <- 0; i < 7; i +<- 1) {
        bool ok <- CanTxQueue.enqueue(i);
        assert(ok = true);
    }
    bool overflow <- CanTxQueue.enqueue(99);
    assert(overflow = false);
}

void test_wraparound() {
    CanTxQueue.reset();
    u8 sink <- 0;
    for (u8 i <- 0; i < 20; i +<- 1) {       // exercise index wrap
        bool ok <- CanTxQueue.enqueue(i);
        assert(ok = true);
        bool got <- CanTxQueue.dequeue(sink);
        assert(got = true);
        assert(sink = i);
    }
}
```

- [ ] **Step 2: Run the test to verify it fails:**

Run: `cnext src/display/can_tx_queue.test.cnx --test`
Expected: FAIL — `can_tx_queue.cnx` / `CanTxQueue` does not exist yet.

> Confirm the exact C-Next test invocation/flag against the c-next repo's own `.test.cnx` examples (`~/code/c-next/tests/`); use whatever runner those use. Adjust this command to match.

- [ ] **Step 3: Create `src/display/can_tx_queue.cnx`:**

```cnx
// Single-producer/single-consumer ring buffer of vane-position bytes.
// Producer = timer ISR (enqueue); consumer = main loop (dequeue).
scope CanTxQueue {
    const u8 CAPACITY <- 8;             // power of two
    const u8 MASK <- 7;                 // CAPACITY - 1
    u8[8] positions;
    atomic u8 head <- 0;               // producer index
    atomic u8 tail <- 0;               // consumer index

    void reset() {
        this.head <- 0;
        this.tail <- 0;
    }

    // Producer: single writer. Drop-newest when full.
    bool enqueue(u8 position) {
        u8 next <- (this.head + 1) & MASK;
        bool full <- (next = this.tail);
        if (full = true) {
            return false;
        }
        this.positions[this.head] <- position;
        this.head <- next;
        return true;
    }

    // Consumer: single reader. critical guards read+advance vs the ISR producer.
    bool dequeue(u8 outPosition) {
        bool empty <- (this.head = this.tail);
        if (empty = true) {
            return false;
        }
        critical {
            outPosition <- this.positions[this.tail];
            this.tail <- (this.tail + 1) & MASK;
        }
        return true;
    }
}
```

- [ ] **Step 4: Run the test to verify it passes:**

Run: `cnext src/display/can_tx_queue.test.cnx --test`
Expected: PASS — all four tests.

- [ ] **Step 5: Commit** (include generated artifacts per project rule):

```bash
git add src/display/can_tx_queue.cnx src/display/can_tx_queue.cpp include/display/CanTxQueue.hpp src/display/can_tx_queue.test.cnx src/display/can_tx_queue.test.c src/display/can_tx_queue.test.h
git commit -m "Add standalone SPSC CanTxQueue scope in C-Next with tests

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 4: Timer producer + Loop consumer (the actual ISR fix)

The 20 ms `IntervalTimer` enqueues the latest demanded position; `Loop()` drains and writes — moving every `canBus.write()` into main context.

**Files:**
- Modify: `src/display/actuator.cnx`

- [ ] **Step 1: Add the queue include, timer, producer ISR, and command helper.** At the top of `actuator.cnx` add the include (alongside the others):

```cnx
#include "can_tx_queue.cnx"
```

Add the timer instance near the CAN instance:

```cnx
    IntervalTimer txTimer;
```

Add the producer ISR (private) and the position→CAN-command helper:

```cnx
    // Timer ISR: enqueue only — no CAN write in interrupt context.
    private void txTimerISR() {
        u8 demanded <- global.appData.actuatorDemandedPosition;
        bool ignored <- CanTxQueue.enqueue(demanded);
    }

    // position (0..100) -> CAN command byte (0..250), closed/open inverted.
    private u8 positionToCommand(u8 position) {
        u32 inverted <- 100 - position;
        u32 command <- inverted * 250 / 100;
        return command[0, 8];
    }
```

- [ ] **Step 2: Start the timer in `Initialize()`** (add after `mailboxStatus()`):

```cnx
        this.txTimer.begin(this.txTimerISR, 20000);   // 20 ms, microseconds
```

- [ ] **Step 3: Implement `Loop()` to drain + write:**

```cnx
    void Loop() {
        u8 position <- 0;
        bool got <- CanTxQueue.dequeue(position);
        while (got = true) {
            CAN_message_t msg;
            msg.id <- CMD_ID;
            msg.flags.extended <- 0;
            msg.len <- 8;
            for (u8 i <- 0; i < 8; i +<- 1) {
                msg.buf[i] <- 0;
            }
            msg.buf[0] <- this.positionToCommand(position);
            this.canBus.write(msg);
            got <- CanTxQueue.dequeue(position);
        }
    }
```

- [ ] **Step 4: Transpile and build:**

Run: `pio run -e teensy41`
Expected: SUCCESS.

> Interop risk: `IntervalTimer` is a C++ class taking a function pointer. If `txTimer.begin(this.txTimerISR, 20000)` does not transpile, the fallback is to **drop the timer entirely and call `CanTxQueue.enqueue(global.appData.actuatorDemandedPosition)` at the top of `Loop()`** (producer and consumer both in main context). That still removes the ISR write — the queue simply becomes single-context. Note this in the conversion doc if used.

- [ ] **Step 5: Commit:**

```bash
git add src/display/actuator.cnx src/display/actuator.cpp include/display/Actuator.hpp
git commit -m "Move actuator CAN write out of ISR via timer-enqueue + Loop-drain

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 5: Full build + on-bench verification

**Files:** none (verification only)

- [ ] **Step 1: Clean build:**

Run: `pio run -e teensy41 -t clean && pio run -e teensy41`
Expected: SUCCESS, no warnings referencing `actuator`.

- [ ] **Step 2: Native test suite still green** (exhaust brake / j1939 decode unaffected):

Run: `pio test -e native`
Expected: PASS — all suites 0 failures.

- [ ] **Step 3: Flash and bench-check** (USB, engine off; close any serial monitor first):

Run: `pio run -e teensy41 -t upload`
Then open `pio device monitor -b 115200`, type `auto`, and confirm: boots cleanly (`Actuator (C-Next) initialized`), `Pos` tracks `Dem`, and manual numbers still move the vanes (`0`..`68`).

---

## Task 6: Static-analysis before/after + conversion log

**Files:**
- Create: `docs/internal/cnext-conversion.md`

- [ ] **Step 1: Capture the after-state static analysis:**

Run: `pio check -e teensy41 --skip-packages --severity=medium 2>&1 | grep -E "src/display|Total|HIGH"`
Record the counts.

- [ ] **Step 2: Write `docs/internal/cnext-conversion.md`** mirroring OSSM's format — a table of cppcheck/clang-tidy findings for `src/display/` before (the hand-written `actuator.cpp`, from git history / the earlier `pio check` run) vs after (the C-Next-generated one), plus a "Conversion Progress" list (actuator = done; next candidates: `j1939`, `boostController`, `ovgt`), and firmware size before/after (`teensy_size` output from the build).

- [ ] **Step 3: Commit:**

```bash
git add docs/internal/cnext-conversion.md
git commit -m "Document actuator C-Next conversion: static-analysis before/after

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Notes for the implementer

- **Give-back rule:** if the transpiler rejects valid-looking C-Next or emits wrong C (especially around the `appData` C++-struct access or `IntervalTimer`), file a c-next issue with a **minimal reproducible example** (per the zero-exceptions bug rule) before working around it. Reference issue #997 if it relates to the queue.
- **Commit generated files** (`.cpp`, `.hpp`, `.test.c/.h`) alongside every `.cnx` change — they are part of the build.
- **Reboot confirmation is on-truck:** this removes the ISR-write suspect, but confirm with `CrashReport` (already in `setup()`) next truck session; if reboots persist with a blank crash report, pivot to the power/brownout track.
