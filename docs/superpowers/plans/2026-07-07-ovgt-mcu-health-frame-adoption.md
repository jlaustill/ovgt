# OVGT MCU-Health Frame Adoption Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** OVGT adopts the `jlaustill/J1939` library and broadcasts the fleet MCU-health frame (PGN `0xFFDC`) at 1 Hz with the four fields it already computes — MCU die temp, reset cause, boot count, uptime.

**Architecture:** Add `jlaustill/J1939` as a PlatformIO dependency. Extract a pure, host-testable helper `buildMcuHealth()` that maps OVGT's controller-health readings into the library's `McuHealth` struct (sentinel-safe capping of the two 16-bit fields). Wire one call into the existing `transmit1000ms()` in `src/sensors/j1939.cpp`, encoding with `J1939McuHealth_encode` and sending through OVGT's existing `sendPgn` helper (which already builds the `0x18FFDC01` extended ID).

**Tech Stack:** C++ (Teensy 4.1 / Arduino / PlatformIO), the `jlaustill/J1939@^2.1.0` C library (McuHealth codec), Unity host tests (`pio test -e native`), FlexCAN_T4.

**Spec:** `docs/superpowers/specs/2026-07-06-mcu-health-j1939-pgn-design.md` (this implements **Subsystem 2** — the OVGT reference firmware. Subsystem 1, the library module, is DONE and published as `jlaustill/J1939@2.1.0`. OSSM/OCT and the listener are separate downstream plans.)

## Global Constraints

- **PGN:** `65500` / `0xFFDC`, Proprietary B, priority 6, **1000 ms** rate. On OVGT (SA `0x01`) the extended ID is `0x18FFDC01`.
- **Payload (8 B, LE):** B0–1 MCU die temp (0.03125 °C/bit, −273 off, `0xFFFF`=N/A); B2 reset cause (u8, `0xFF`=not-impl); B3–4 boot count (u16, `0xFFFF`=N/A); B5–6 uptime minutes (u16, `0xFFFF`=N/A); B7 reserved `0xFF`. **All encoding lives in the library** — OVGT only populates the `McuHealth` struct.
- **Reset-cause codes:** OVGT's `SystemHealth_resetCauseCode()` already returns `0..8` in the exact library ordering (both derive from the i.MX RT1062 `SRSR` decode) — pass it through verbatim.
- **Library dependency:** `jlaustill/J1939` (published 2.1.0). OVGT retires its hand-rolled J1939 **for this frame only** — the existing turbo/engine TX in `j1939.cpp` is untouched.
- **Sentinel safety:** boot count and uptime are `uint32_t` sources narrowed to `uint16_t`; cap at `0xFFFE` so a genuinely large value never collides with the `0xFFFF` N/A sentinel.
- **Build/test:** firmware `pio run -e teensy41`; host tests `pio test -e native` (Unity), test dirs are `test/test_<name>/test_<name>.cpp`; a newly host-tested `src/` file must be added to `[env:native]` `build_src_filter`.

## File Structure

- `platformio.ini` — add `jlaustill/J1939` to `[env:teensy41]` and `[env:native]` `lib_deps`; add `sensors/mcuHealthFrame.cpp` to `[env:native]` `build_src_filter`.
- `src/sensors/mcuHealthFrame.h` / `.cpp` (**new**) — pure `buildMcuHealth()` mapping helper. One responsibility: raw readings → `McuHealth` struct with sentinel-safe capping. No hardware calls (fully host-testable).
- `test/test_mcu_health_frame/test_mcu_health_frame.cpp` (**new**) — Unity host test for the mapping + encode.
- `src/sensors/j1939.cpp` (**modify**) — include the helper + library header + `systemHealth.h`; in `transmit1000ms()` read the four accessors, build, encode, `sendPgn`.

---

### Task 1: Pure `buildMcuHealth()` helper + dependency wiring (host-tested)

**Files:**
- Modify: `platformio.ini` (`[env:teensy41]` + `[env:native]` `lib_deps`; `[env:native]` `build_src_filter`)
- Create: `src/sensors/mcuHealthFrame.h`, `src/sensors/mcuHealthFrame.cpp`
- Test: `test/test_mcu_health_frame/test_mcu_health_frame.cpp`

**Interfaces:**
- Consumes: `McuHealth` struct + `J1939McuHealth_encode(const McuHealth*, uint8_t[8])` from `jlaustill/J1939@2.1.0`.
- Produces (consumed by Task 2):
  - `McuHealth buildMcuHealth(float mcuTempC, uint8_t resetCauseCode, uint32_t bootCount, uint32_t uptimeMinutes)` — returns a fully-populated, all-valid `McuHealth`; caps `bootCount`/`uptimeMinutes` at `0xFFFE`; passes `resetCauseCode` through verbatim.

- [ ] **Step 1: Write the failing test**

Create `test/test_mcu_health_frame/test_mcu_health_frame.cpp`:

```cpp
#include <unity.h>
#include "sensors/mcuHealthFrame.h"
#include <J1939McuHealth.h>

void setUp(void) {}
void tearDown(void) {}

// Maps fields and encodes to the expected 8 bytes (temp 60C, wdog, boot 7, uptime 1000).
void test_build_and_encode(void) {
    McuHealth h = buildMcuHealth(60.0f, 4 /*wdog*/, 7, 1000);
    uint8_t buf[8];
    J1939McuHealth_encode(&h, buf);
    TEST_ASSERT_EQUAL_UINT8(0xA0, buf[0]);   // 60C -> raw 0x29A0, LE
    TEST_ASSERT_EQUAL_UINT8(0x29, buf[1]);
    TEST_ASSERT_EQUAL_UINT8(4,    buf[2]);
    TEST_ASSERT_EQUAL_UINT8(0x07, buf[3]);
    TEST_ASSERT_EQUAL_UINT8(0x00, buf[4]);
    TEST_ASSERT_EQUAL_UINT8(0xE8, buf[5]);   // 1000 -> 0x03E8, LE
    TEST_ASSERT_EQUAL_UINT8(0x03, buf[6]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, buf[7]);
}

// u32 sources cap at 0xFFFE so a real value never masquerades as the 0xFFFF N/A sentinel.
void test_caps_below_sentinel(void) {
    McuHealth h = buildMcuHealth(20.0f, 0, 0xFFFFFFFFu, 0x10000u);
    TEST_ASSERT_EQUAL_UINT16(0xFFFE, h.bootCount);
    TEST_ASSERT_EQUAL_UINT16(0xFFFE, h.uptimeMinutes);
    TEST_ASSERT_TRUE(h.bootCountValid);
    TEST_ASSERT_TRUE(h.uptimeValid);
    TEST_ASSERT_TRUE(h.temperatureValid);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_build_and_encode);
    RUN_TEST(test_caps_below_sentinel);
    return UNITY_END();
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `pio test -e native -f test_mcu_health_frame`
Expected: FAIL — `sensors/mcuHealthFrame.h` not found / `J1939McuHealth.h` not found (dependency + helper absent).

- [ ] **Step 3: Add the dependency and the native build entry**

In `platformio.ini`, add `jlaustill/J1939` to the `[env:teensy41]` `lib_deps` list (after `SPI`):

```ini
	Wire
	SPI
	jlaustill/J1939
```

Add the same dependency and the new source file to `[env:native]`:

```ini
[env:native]
platform = native
test_framework = unity
test_build_src = yes
build_src_filter = -<*> +<control/exhaustBrakeLogic.cpp> +<control/boostBprLogic.cpp> +<sensors/j1939Decode.cpp> +<sensors/j1939Health.cpp> +<sensors/j1939Encode.cpp> +<sensors/compressorEfficiency.cpp> +<sensors/cotSettle.cpp> +<sensors/mcuHealthFrame.cpp> +<domain/json.c> +<domain/systemHealthLogic.c>
build_flags = -I src -I include
lib_deps = jlaustill/J1939
```

- [ ] **Step 4: Write the helper header**

Create `src/sensors/mcuHealthFrame.h`:

```cpp
#ifndef MCU_HEALTH_FRAME_H
#define MCU_HEALTH_FRAME_H

#include <stdint.h>
#include <J1939McuHealth.h>   // McuHealth struct + codec (jlaustill/J1939)

// Pure mapping: raw controller-health readings -> a fully-valid McuHealth.
// The two 16-bit fields are capped at 0xFFFE so a genuinely large boot count or
// uptime can never read as the 0xFFFF "not available" sentinel. resetCauseCode
// (0..8, already in the library's ordering) is passed through verbatim.
McuHealth buildMcuHealth(float mcuTempC, uint8_t resetCauseCode,
                         uint32_t bootCount, uint32_t uptimeMinutes);

#endif  // MCU_HEALTH_FRAME_H
```

- [ ] **Step 5: Write the helper implementation**

Create `src/sensors/mcuHealthFrame.cpp`:

```cpp
#include "sensors/mcuHealthFrame.h"

// Narrow a u32 reading to u16 without ever hitting the 0xFFFF N/A sentinel.
static uint16_t capBelowSentinel(uint32_t value) {
    return value > 0xFFFEu ? 0xFFFEu : static_cast<uint16_t>(value);
}

McuHealth buildMcuHealth(float mcuTempC, uint8_t resetCauseCode,
                         uint32_t bootCount, uint32_t uptimeMinutes) {
    McuHealth h;
    h.temperatureC     = mcuTempC;
    h.temperatureValid = true;
    h.resetCause       = resetCauseCode;              // 0..8 matches library codes
    h.bootCount        = capBelowSentinel(bootCount);
    h.bootCountValid   = true;
    h.uptimeMinutes    = capBelowSentinel(uptimeMinutes);
    h.uptimeValid      = true;
    return h;
}
```

- [ ] **Step 6: Run the test to verify it passes**

Run: `pio test -e native -f test_mcu_health_frame`
Expected: PASS — both tests green (PlatformIO fetches `jlaustill/J1939@2.1.0` for the native env on first run).

- [ ] **Step 7: Commit**

```bash
cd ~/code/ovgt
git add platformio.ini src/sensors/mcuHealthFrame.h src/sensors/mcuHealthFrame.cpp test/test_mcu_health_frame/test_mcu_health_frame.cpp
git commit -m "feat(telemetry): buildMcuHealth() maps controller health to McuHealth

Adopt jlaustill/J1939 (2.1.0); pure host-tested helper maps MCU temp +
reset cause + boot count + uptime into the library McuHealth struct with
sentinel-safe u16 capping."
```

---

### Task 2: Broadcast the health frame at 1 Hz

**Files:**
- Modify: `src/sensors/j1939.cpp` (includes near line 7; `transmit1000ms()` body — insert after the `PGN_TURBO_INFO_3` send)

**Interfaces:**
- Consumes: `buildMcuHealth(...)` (Task 1); `J1939McuHealth_encode`, `J1939McuHealth_PGN` (library); OVGT's existing `static void sendPgn(uint32_t pgn, const uint8_t* data)`; `tempmonGetTemp()` (Arduino core), `SystemHealth_resetCauseCode()`, `SystemHealth_bootCount()` (`systemHealth.h`), `millis()`.
- Produces: the `0x18FFDC01` frame on CAN2 every 1000 ms. No new symbols for later tasks.

- [ ] **Step 1: Add the includes**

In `src/sensors/j1939.cpp`, after the existing include block (currently ends at `#include "j1939Encode.h"`), add:

```cpp
#include "systemHealth.h"
#include <J1939McuHealth.h>
#include "mcuHealthFrame.h"
```

- [ ] **Step 2: Emit the health frame in `transmit1000ms()`**

In `src/sensors/j1939.cpp`, inside `transmit1000ms()`, immediately after the existing final send `sendPgn(PGN_TURBO_INFO_3, buf);` (and before the closing `}`), add:

```cpp
    // MCU health — PGN 0xFFDC (Proprietary B, broadcast). Controller die temp +
    // reset cause + boot count + uptime, so a listener collects one record per MCU
    // by source address. Payload encoding lives in jlaustill/J1939 (McuHealth).
    McuHealth health = buildMcuHealth(tempmonGetTemp(),
                                      SystemHealth_resetCauseCode(),
                                      SystemHealth_bootCount(),
                                      millis() / 60000UL);
    uint8_t healthBuf[8];
    J1939McuHealth_encode(&health, healthBuf);
    sendPgn(J1939McuHealth_PGN, healthBuf);
```

- [ ] **Step 3: Verify the firmware builds**

Run: `pio run -e teensy41`
Expected: SUCCESS — links `jlaustill/J1939`, no errors. (This is the automated gate; the TX path itself is hardware and cannot be host-tested.)

- [ ] **Step 4: Commit**

```bash
cd ~/code/ovgt
git add src/sensors/j1939.cpp
git commit -m "feat(telemetry): broadcast MCU-health frame (PGN 0xFFDC) at 1 Hz

Populate McuHealth from tempmonGetTemp + SystemHealth reset-cause/boot-count
+ uptime minutes, encode via jlaustill/J1939, send on CAN2 as 0x18FFDC01."
```

- [ ] **Step 5: On-bus verification (manual, on-truck / bench with CAN2 wired)**

Flash: `pio run -e teensy41 -t upload`, then on a CAN tool watching the 250 kbps bus:
Run: `candump can0 | grep 18FFDC01` (or the equivalent on your analyzer)
Expected: an `18FFDC01` frame ~once per second. Sanity-decode: bytes 0–1 little-endian → `(raw/32)−273` should match the `mcu_c` value in the serial NDJSON telemetry; byte 2 should match the numeric `reset_cause`; bytes 3–4 → `boot_count`. This is a real-world check, not a commit gate.

---

## Self-Review

**Spec coverage (Subsystem 2 — OVGT reference firmware):**
- OVGT adopts `jlaustill/J1939` → Task 1 (`lib_deps` in both envs). ✅
- Broadcast PGN `0xFFDC` at 1000 ms with OVGT's SA → Task 2 (in `transmit1000ms()`, via `sendPgn` → `0x18FFDC01`). ✅
- Populate temp + reset cause + boot count + uptime → Task 1 `buildMcuHealth` from `tempmonGetTemp`/`SystemHealth_resetCauseCode`/`SystemHealth_bootCount`/`millis()/60000`. ✅
- Encoding stays in the library → Task 2 calls `J1939McuHealth_encode`; OVGT never touches bytes. ✅
- Sentinel safety on u32→u16 narrowing → Task 1 `capBelowSentinel` + `test_caps_below_sentinel`. ✅
- Reset-cause code passthrough (already matches library ordering) → Task 1 verbatim; Global Constraints note. ✅
- Existing turbo/engine TX untouched → Task 2 only appends to `transmit1000ms()`. ✅

**Placeholder scan:** No TBD/TODO; every code step shows complete code; every run step names the exact command and expected result. The one non-automated step (2.5, on-bus) is explicitly labelled manual and not a commit gate. ✅

**Type consistency:** `buildMcuHealth(float, uint8_t, uint32_t, uint32_t) -> McuHealth` is identical across the Task 1 header, implementation, test, and the Task 2 call site. `McuHealth` field names (`temperatureC`, `resetCause`, `bootCount`, `uptimeMinutes`, `*Valid`) match the library header used in Subsystem 1. `J1939McuHealth_encode`/`J1939McuHealth_PGN` match the published library. ✅
