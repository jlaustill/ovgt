# J1939 `McuHealth` Library Module Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a canonical `J1939McuHealth` codec module to the `jlaustill/J1939` library (v2 line) that encodes/decodes the 8-byte MCU-health payload for Proprietary B PGN `0xFFDC` (65500), and publish it to the PlatformIO registry.

**Architecture:** A sans-I/O pure module in C-Next (`src/J1939McuHealth.cnx`), transpiled to `src/J1939McuHealth.c` + `include/J1939McuHealth.h`. It exposes a plain `McuHealth` struct plus `J1939McuHealth` scope functions `encode` (struct → 8 bytes) and `decode` (8 bytes → struct), and `public const` constants for the PGN, transmit interval, and reset-cause codes. No CAN I/O and no time source — CAN-ID assembly and transmission belong to the consuming firmware.

**Tech Stack:** C-Next (`cnext` transpiler), C, GoogleTest (via `pio test`, `platform = native`), PlatformIO library registry.

**Spec:** `docs/superpowers/specs/2026-07-06-mcu-health-j1939-pgn-design.md` (this plan implements only **Subsystem 1** of that spec — the library module. OVGT/OSSM/OCT firmware and the listener/decoder are separate downstream plans that depend on this module being published.)

## Global Constraints

- **Target repo:** `~/code/J1939`, branch **`main`** (v2.x line). Do NOT do this work on `v3-codec-protocol`. Ships ahead of and independent of the v3 rewrite.
- **PGN:** `65500` / `0xFFDC`, Proprietary B (PDU2, broadcast), priority 6, transmit rate 1000 ms.
- **Payload (8 bytes, little-endian):** B0–1 MCU die temp (0.03125 °C/bit, offset −273, `0xFFFF`=N/A); B2 reset cause (u8 enum below, `0xFF`=not-implemented); B3–4 boot count (u16, `0xFFFF`=N/A); B5–6 uptime minutes (u16, `0xFFFF`=N/A); B7 reserved `0xFF`.
- **Reset-cause codes (verbatim from OVGT `systemHealthLogic.cnx`, i.MX RT1062 SRSR decode):** `0` unknown, `1` por, `2` lockup, `3` pin, `4` wdog, `5` wdog3, `6` tempsense, `7` jtag, `8` csu. `0xFF` = reset-cause not implemented on this MCU.
- **C-Next rules:** assignment `<-`, equality `=`; edit `.cnx` and transpile with `cnext src/J1939McuHealth.cnx` — never hand-edit generated `.c`/`.h`; commit generated files. Never work around a C-Next bug — fix upstream in the c-next repo (per user policy).
- **C-Next gotchas that apply here:** cannot shift a `u8` by 8 (MISRA 12.2) — assemble/split u16 via bit-indexing on a scalar (`raw[0,8]`, `raw[8,8]`), not `<<`; float→int uses `(u16)expr` (truncate+clamp); a byte-array single subscript `data[i]` yields a `u8`.
- **Test harness:** GoogleTest C++ in `test/`, all tests run together via `pio test` (no single-test runner). Tests call the generated C API (`J1939McuHealth_encode`, `J1939McuHealth_decode`, struct `McuHealth`).

---

### Task 1: `McuHealth` struct, constants, and `encode()`

**Files:**
- Create: `~/code/J1939/src/J1939McuHealth.cnx`
- Generate (via transpile, then commit): `~/code/J1939/src/J1939McuHealth.c`, `~/code/J1939/include/J1939McuHealth.h`
- Create: `~/code/J1939/test/J1939McuHealth_test.cpp`

**Interfaces:**
- Produces (consumed by Task 2 and by all downstream firmware/listener plans):
  - `struct McuHealth { float temperatureC; uint8_t resetCause; uint16_t bootCount; uint16_t uptimeMinutes; bool temperatureValid; bool bootCountValid; bool uptimeValid; }`
  - `void J1939McuHealth_encode(McuHealth* health, uint8_t out[8])` — fills 8 bytes, writing the N/A sentinel for any field whose `*Valid` flag is false (and `0xFF` reserved byte 7).
  - Constants in the generated header: `J1939McuHealth_PGN` (=`0xFFDC`), `J1939McuHealth_TRANSMIT_INTERVAL_MS` (=`1000`), and reset-cause codes `J1939McuHealth_RESET_UNKNOWN`=0 … `J1939McuHealth_RESET_CSU`=8, `J1939McuHealth_RESET_NOT_IMPLEMENTED`=`0xFF`.

- [ ] **Step 1: Write the failing test**

Create `~/code/J1939/test/J1939McuHealth_test.cpp`:

```cpp
#include <J1939McuHealth.h>
#include <gtest/gtest.h>

// A fully-populated, all-valid health record.
static McuHealth makeFull() {
  McuHealth h;
  h.temperatureC = 60.0f;      // (60 + 273) * 32 = 10656 = 0x29A0
  h.resetCause = J1939McuHealth_RESET_WDOG;   // 4
  h.bootCount = 7;
  h.uptimeMinutes = 1000;
  h.temperatureValid = true;
  h.bootCountValid = true;
  h.uptimeValid = true;
  return h;
}

TEST(J1939McuHealth, pgnConstant) {
  EXPECT_EQ(J1939McuHealth_PGN, 0xFFDC);
  EXPECT_EQ(J1939McuHealth_TRANSMIT_INTERVAL_MS, 1000);
}

TEST(J1939McuHealth, encodeFullFrame) {
  McuHealth h = makeFull();
  uint8_t out[8] = {0};
  J1939McuHealth_encode(&h, out);

  // Temp 60C -> raw 10656 (0x29A0), little-endian.
  EXPECT_EQ(out[0], 0xA0);
  EXPECT_EQ(out[1], 0x29);
  // Reset cause wdog = 4.
  EXPECT_EQ(out[2], 4);
  // Boot count 7, little-endian.
  EXPECT_EQ(out[3], 0x07);
  EXPECT_EQ(out[4], 0x00);
  // Uptime 1000 min = 0x03E8, little-endian.
  EXPECT_EQ(out[5], 0xE8);
  EXPECT_EQ(out[6], 0x03);
  // Reserved.
  EXPECT_EQ(out[7], 0xFF);
}

TEST(J1939McuHealth, encodeSentinelsForInvalidFields) {
  McuHealth h = makeFull();
  h.temperatureValid = false;
  h.bootCountValid = false;
  h.uptimeValid = false;
  h.resetCause = J1939McuHealth_RESET_NOT_IMPLEMENTED;  // 0xFF
  uint8_t out[8] = {0};
  J1939McuHealth_encode(&h, out);

  EXPECT_EQ(out[0], 0xFF);
  EXPECT_EQ(out[1], 0xFF);
  EXPECT_EQ(out[2], 0xFF);
  EXPECT_EQ(out[3], 0xFF);
  EXPECT_EQ(out[4], 0xFF);
  EXPECT_EQ(out[5], 0xFF);
  EXPECT_EQ(out[6], 0xFF);
  EXPECT_EQ(out[7], 0xFF);
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd ~/code/J1939 && pio test -e latest_stable`
Expected: FAIL — link/compile error, `J1939McuHealth.h` not found / `J1939McuHealth_encode` undefined.

- [ ] **Step 3: Write the C-Next source**

Create `~/code/J1939/src/J1939McuHealth.cnx`:

```
// MCU Health frame codec — Proprietary B PGN 0xFFDC (65500), broadcast @ 1 Hz.
// Every fleet MCU (OVGT/OSSM/OCT, all Teensy 4.x) transmits this with its own
// source address; a listener collects records keyed by SA.
// Contract spec: ovgt docs/superpowers/specs/2026-07-06-mcu-health-j1939-pgn-design.md

struct McuHealth {
    f32 temperatureC;
    u8  resetCause;
    u16 bootCount;
    u16 uptimeMinutes;
    bool temperatureValid;
    bool bootCountValid;
    bool uptimeValid;
}

scope J1939McuHealth {
    public const u16 PGN <- 0xFFDC;
    public const u16 TRANSMIT_INTERVAL_MS <- 1000;

    // Reset-cause codes (i.MX RT1062 SRSR decode; matches OVGT systemHealthLogic).
    public const u8 RESET_UNKNOWN         <- 0;
    public const u8 RESET_POR             <- 1;
    public const u8 RESET_LOCKUP          <- 2;
    public const u8 RESET_PIN             <- 3;
    public const u8 RESET_WDOG            <- 4;
    public const u8 RESET_WDOG3           <- 5;
    public const u8 RESET_TEMPSENSE       <- 6;
    public const u8 RESET_JTAG            <- 7;
    public const u8 RESET_CSU             <- 8;
    public const u8 RESET_NOT_IMPLEMENTED <- 0xFF;

    void encode(McuHealth health, u8[8] out) {
        // B0-1: MCU die temp, 0.03125 C/bit, offset -273, LE.
        if (health.temperatureValid = true) {
            u16 tempRaw <- (u16)((health.temperatureC + 273.0) * 32.0);
            out[0] <- tempRaw[0, 8];
            out[1] <- tempRaw[8, 8];
        } else {
            out[0] <- 0xFF;
            out[1] <- 0xFF;
        }

        // B2: reset cause (u8 passthrough; 0xFF = not implemented).
        out[2] <- health.resetCause;

        // B3-4: boot count, LE.
        if (health.bootCountValid = true) {
            out[3] <- health.bootCount[0, 8];
            out[4] <- health.bootCount[8, 8];
        } else {
            out[3] <- 0xFF;
            out[4] <- 0xFF;
        }

        // B5-6: uptime minutes, LE.
        if (health.uptimeValid = true) {
            out[5] <- health.uptimeMinutes[0, 8];
            out[6] <- health.uptimeMinutes[8, 8];
        } else {
            out[5] <- 0xFF;
            out[6] <- 0xFF;
        }

        // B7: reserved.
        out[7] <- 0xFF;
    }
}
```

- [ ] **Step 4: Transpile C-Next → C**

Run: `cd ~/code/J1939 && cnext src/J1939McuHealth.cnx`
Expected: generates `src/J1939McuHealth.c` and `include/J1939McuHealth.h`, no errors.
If `cnext` errors on valid syntax, STOP and fix it upstream in the c-next repo (per user policy) — do not hand-edit generated files or rewrite around it.

- [ ] **Step 5: Run the test to verify it passes**

Run: `cd ~/code/J1939 && pio test -e latest_stable`
Expected: PASS — `J1939McuHealth.pgnConstant`, `encodeFullFrame`, `encodeSentinelsForInvalidFields` all green; existing `J1939Message` tests still green.

- [ ] **Step 6: Commit**

```bash
cd ~/code/J1939
git add src/J1939McuHealth.cnx src/J1939McuHealth.c include/J1939McuHealth.h test/J1939McuHealth_test.cpp
git commit -m "feat(mcu-health): McuHealth struct, constants, and encode()

Proprietary B PGN 0xFFDC codec — struct + encode with per-field N/A
sentinels. Canonical fleet contract; ships on v2 line ahead of v3."
```

---

### Task 2: `decode()` and encode/decode round-trip

**Files:**
- Modify: `~/code/J1939/src/J1939McuHealth.cnx` (add `decode` to the `J1939McuHealth` scope)
- Regenerate (then commit): `~/code/J1939/src/J1939McuHealth.c`, `~/code/J1939/include/J1939McuHealth.h`
- Modify: `~/code/J1939/test/J1939McuHealth_test.cpp` (add decode + round-trip tests)

**Interfaces:**
- Consumes: `McuHealth`, `J1939McuHealth_encode` (Task 1).
- Produces (consumed by the downstream listener/decoder plan):
  - `void J1939McuHealth_decode(uint8_t data[8], McuHealth* health)` — fills `health` from 8 bytes; sets each `*Valid` flag false when its field holds the N/A sentinel (`temperatureC=0`, `bootCount=0`, `uptimeMinutes=0` when invalid); `resetCause` is copied through verbatim (including `0xFF`).

- [ ] **Step 1: Write the failing test**

Append to `~/code/J1939/test/J1939McuHealth_test.cpp`:

```cpp
TEST(J1939McuHealth, decodeFullFrame) {
  // Temp raw 0x29A0 (60C), wdog(4), boot 7, uptime 1000 (0x03E8), reserved 0xFF.
  uint8_t data[8] = {0xA0, 0x29, 4, 0x07, 0x00, 0xE8, 0x03, 0xFF};
  McuHealth h;
  J1939McuHealth_decode(data, &h);

  EXPECT_TRUE(h.temperatureValid);
  EXPECT_NEAR(h.temperatureC, 60.0f, 0.02f);
  EXPECT_EQ(h.resetCause, 4);
  EXPECT_TRUE(h.bootCountValid);
  EXPECT_EQ(h.bootCount, 7);
  EXPECT_TRUE(h.uptimeValid);
  EXPECT_EQ(h.uptimeMinutes, 1000);
}

TEST(J1939McuHealth, decodeSentinelsMarkInvalid) {
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  McuHealth h;
  J1939McuHealth_decode(data, &h);

  EXPECT_FALSE(h.temperatureValid);
  EXPECT_EQ(h.resetCause, J1939McuHealth_RESET_NOT_IMPLEMENTED);  // 0xFF passthrough
  EXPECT_FALSE(h.bootCountValid);
  EXPECT_FALSE(h.uptimeValid);
}

TEST(J1939McuHealth, encodeThenDecodeRoundTrip) {
  McuHealth src = makeFull();
  uint8_t buf[8] = {0};
  J1939McuHealth_encode(&src, buf);

  McuHealth out;
  J1939McuHealth_decode(buf, &out);

  EXPECT_TRUE(out.temperatureValid);
  EXPECT_NEAR(out.temperatureC, src.temperatureC, 0.02f);
  EXPECT_EQ(out.resetCause, src.resetCause);
  EXPECT_EQ(out.bootCount, src.bootCount);
  EXPECT_EQ(out.uptimeMinutes, src.uptimeMinutes);
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd ~/code/J1939 && pio test -e latest_stable`
Expected: FAIL — `J1939McuHealth_decode` undefined.

- [ ] **Step 3: Add `decode` to the C-Next source**

Insert this function into the `J1939McuHealth` scope in `~/code/J1939/src/J1939McuHealth.cnx` (after `encode`):

```
    void decode(u8[8] data, McuHealth health) {
        // B0-1: MCU die temp. Assemble LE u16 via bit-indexing (no u8<<8).
        u16 tempRaw <- 0;
        tempRaw[0, 8] <- data[0];
        tempRaw[8, 8] <- data[1];
        if (tempRaw >= 0xFF00) {
            health.temperatureValid <- false;
            health.temperatureC <- 0.0;
        } else {
            health.temperatureValid <- true;
            health.temperatureC <- (f32)tempRaw / 32.0 - 273.0;
        }

        // B2: reset cause, verbatim.
        health.resetCause <- data[2];

        // B3-4: boot count.
        u16 boot <- 0;
        boot[0, 8] <- data[3];
        boot[8, 8] <- data[4];
        if (boot = 0xFFFF) {
            health.bootCountValid <- false;
            health.bootCount <- 0;
        } else {
            health.bootCountValid <- true;
            health.bootCount <- boot;
        }

        // B5-6: uptime minutes.
        u16 up <- 0;
        up[0, 8] <- data[5];
        up[8, 8] <- data[6];
        if (up = 0xFFFF) {
            health.uptimeValid <- false;
            health.uptimeMinutes <- 0;
        } else {
            health.uptimeValid <- true;
            health.uptimeMinutes <- up;
        }
    }
```

- [ ] **Step 4: Transpile C-Next → C**

Run: `cd ~/code/J1939 && cnext src/J1939McuHealth.cnx`
Expected: regenerates `.c`/`.h`, no errors. (Upstream-fix policy from Task 1 Step 4 still applies.)

- [ ] **Step 5: Run the test to verify it passes**

Run: `cd ~/code/J1939 && pio test -e latest_stable`
Expected: PASS — all `J1939McuHealth` tests green (encode, decode, round-trip), `J1939Message` tests still green.

- [ ] **Step 6: Commit**

```bash
cd ~/code/J1939
git add src/J1939McuHealth.cnx src/J1939McuHealth.c include/J1939McuHealth.h test/J1939McuHealth_test.cpp
git commit -m "feat(mcu-health): decode() + encode/decode round-trip tests"
```

---

### Task 3: Publish the module to the PlatformIO registry

**Files:**
- Modify: `~/code/J1939/library.json` (version bump)
- Modify: `~/code/J1939/publish.sh` (transpile the new module too)
- Modify: `~/code/J1939/CLAUDE.md` and `~/code/J1939/README.md` (document the new module)

**Interfaces:**
- Consumes: the built + tested module from Tasks 1–2.
- Produces: a published `jlaustill/J1939` registry version that OSSM/OCT (and OVGT, once it adopts the dep) can pull.

- [ ] **Step 1: Bump the library version**

Edit `~/code/J1939/library.json`: change `"version": "2.0.0"` to `"version": "2.1.0"` (new backward-compatible module → minor bump).

- [ ] **Step 2: Add the new module to `publish.sh`**

Edit `~/code/J1939/publish.sh` — add a transpile line for the new module alongside the existing one, so a clean publish regenerates both:

```bash
# Transpile C-Next source to C
cnext src/J1939Message.cnx
cnext src/J1939McuHealth.cnx
```

- [ ] **Step 3: Document the module**

In `~/code/J1939/CLAUDE.md`, under Architecture, add a short subsection describing `J1939McuHealth` (PGN `0xFFDC`, the 8-byte layout, `encode`/`decode`, reset-cause codes, N/A sentinels). In `~/code/J1939/README.md`, add a usage snippet showing `J1939McuHealth_encode`/`decode` and the `McuHealth` struct.

- [ ] **Step 4: Verify a clean transpile + full test pass**

Run: `cd ~/code/J1939 && cnext src/J1939Message.cnx && cnext src/J1939McuHealth.cnx && pio test -e latest_stable`
Expected: PASS — full suite green (existing `J1939Message` tests + new `J1939McuHealth` tests).

- [ ] **Step 5: Commit**

```bash
cd ~/code/J1939
git add library.json publish.sh CLAUDE.md README.md src/J1939McuHealth.c include/J1939McuHealth.h
git commit -m "chore(release): J1939 2.1.0 — add McuHealth module"
```

- [ ] **Step 6: Publish (requires PlatformIO auth — confirm with the user first)**

Run: `cd ~/code/J1939 && ./publish.sh`
Expected: transpiles, `pio test` passes, then `pio pkg publish` uploads `jlaustill/J1939@2.1.0`.
Note: publishing is an outward-facing, hard-to-reverse action and needs `pio account` auth — pause and get explicit user confirmation before running this step. Pushing the `main` commits to GitHub is likewise user-gated.

---

## Self-Review

**Spec coverage (Subsystem 1 only):**
- Transport constants (PGN `0xFFDC`, 1000 ms) → Task 1 (`PGN`, `TRANSMIT_INTERVAL_MS`, tested in `pgnConstant`). ✅
- Payload layout + temp encoding + LE + reserved byte → Task 1 `encode` + `encodeFullFrame`. ✅
- Per-field N/A sentinels → Task 1 `encodeSentinelsForInvalidFields`, Task 2 `decodeSentinelsMarkInvalid`. ✅
- Reset-cause enum (canonical) → Task 1 `RESET_*` constants (values 0–8 + `0xFF`). ✅
- decode for the listener → Task 2 `decode` + round-trip. ✅
- Canonical home = `jlaustill/J1939` on `main`, published ahead of v3 → Task 3. ✅
- Firmware (OVGT/OSSM/OCT) and listener wiring are explicitly **out of scope** for this plan (downstream plans) — spec's rollout steps 2–4. Noted, not a gap.

**Placeholder scan:** No TBD/TODO; every code step shows complete `.cnx`/C++ code; every run step names the exact command and expected result. ✅

**Type consistency:** `McuHealth` field names (`temperatureC`, `resetCause`, `bootCount`, `uptimeMinutes`, `temperatureValid`, `bootCountValid`, `uptimeValid`) are identical across Task 1 struct, Task 1 encode, Task 2 decode, and all tests. Function names `J1939McuHealth_encode`/`J1939McuHealth_decode` and constant names `J1939McuHealth_PGN`/`RESET_*` are consistent throughout. ✅
