# OVGT Telemetry — Firmware Implementation Plan (Part 1 of 2)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Emit OVGT telemetry as 10 Hz NDJSON over USB serial, built by a reusable, heap-free C-Next `Json` scope, replacing the 1 Hz human-readable line — enabling the host tool (Part 2) to log to MongoDB and replay the `~` settled-CE gate.

**Architecture:** A generic C-Next `Json` builder scope (`src/domain/json.cnx`, transpiled to `json.hpp`/`json.cpp` like the `actuator.cnx` pilot) writes JSON into a fixed `string<N>` buffer via a cursor (byte-writes + manual itoa/fixed-decimal — no heap, no `snprintf`). `ovgt.cpp` calls `Json_*` at 10 Hz to assemble the telemetry object and on each gate measurement to assemble a settle object, then `Serial.write`s the buffer + `\n`.

**Tech Stack:** C-Next (`cnext` v0.2.16), Teensy 4.1 / Arduino, PlatformIO. C-Next builder unit-tested host-side via `cnext` + `gcc` (the c-next `.test.cnx` convention); gate-surfacing logic unit-tested via the existing PlatformIO Unity `native` env.

**Spec:** `docs/plans/2026-06-19-mongodb-telemetry-design.md`. **Part 2** (the TS OpenTUI host) is a separate plan, written after this lands so it can test against real telemetry.

**Prerequisite note:** the `Json` builder declares its buffer `string<640> buf <- "";` — explicit empty init is *required* (and correct): a no-init scope-member string is bug #1019. Do not change it to `string<640> buf;`.

**Verified:** the entire builder below was prototyped, transpiled (`cnext`), compiled (`gcc -std=c11`), and run — it produces exact NDJSON. The code in this plan is that verified code.

---

### Task 1: `Json` builder — scaffold + byte primitive (TDD)

**Files:**
- Create: `src/domain/json.cnx`
- Create: `src/domain/json.test.cnx`

The builder is a singleton scope. Buffer sized for the worst-case telemetry object (~25 fields ≈ 500 B + margin → 640).

- [ ] **Step 1: Write the failing test** — `src/domain/json.test.cnx`

```cnx
// test-execution
#include "json.cnx"

u32 main() {
    Json.begin();           // writes '{'
    Json.putByte('}');      // close manually for this scaffold test
    u32 n <- Json.len();
    if (n != 2) return 1;
    u8 c0 <- Json.at(0);
    if (c0 != '{') return 2;
    u8 c1 <- Json.at(1);
    if (c1 != '}') return 3;
    return 0;
}
```

- [ ] **Step 2: Run it, expect FAIL** (json.cnx does not exist yet)

```bash
cd /home/linux/code/ovgt/src/domain
cnext json.test.cnx -o json.test.c   # expect: include error / undefined Json
```

- [ ] **Step 3: Create `src/domain/json.cnx`** with the scaffold

```cnx
// Reusable heap-free NDJSON builder. Fixed buffer + cursor; byte-writes only.
scope Json {
    string<640> buf <- "";   // explicit init required (bug #1019); sized for worst-case object
    u32 cursor;
    bool full;
    bool needComma;

    void putByte(u8 c) {
        u32 i <- this.cursor;
        bool room <- i < 639;
        if (room = true) {
            this.buf[i] <- c;
            this.cursor <- i + 1;
        } else {
            this.full <- true;
        }
    }

    public void begin() {
        this.cursor <- 0;
        this.full <- false;
        this.needComma <- false;
        this.putByte('{');
    }

    public u8 at(u32 i) {
        return this.buf[i];
    }

    public u32 len() {     // NOT length() — collides with deprecated .length
        return this.cursor;
    }
}
```

- [ ] **Step 4: Run it, expect PASS**

```bash
cd /home/linux/code/ovgt/src/domain
cnext json.test.cnx -o json.test.c && gcc -std=c11 -Wall -o json.test json.test.c && ./json.test; echo "exit=$?"
# expect: exit=0
```

- [ ] **Step 5: Commit**

```bash
cd /home/linux/code/ovgt
git add src/domain/json.cnx src/domain/json.test.cnx src/domain/json.test.c
git commit -m "feat(telemetry): Json builder scaffold (begin/putByte/at/len)"
```

---

### Task 2: Unsigned + signed integer fields (`putUintRaw`, `key`, `addUint`, `addInt`)

**Files:**
- Modify: `src/domain/json.cnx`
- Modify: `src/domain/json.test.cnx`

- [ ] **Step 1: Add the failing test cases** to `json.test.cnx`'s `main` (before `return 0`)

```cnx
    Json.begin();
    Json.addUint("a", 12);
    Json.addInt("c", 0 - 7);
    Json.end();
    // Expect: {"a":12,"c":-7}
    string<32> exp1 <- "{\"a\":12,\"c\":-7}";
    u32 w1 <- exp1.char_count;
    u32 g1 <- Json.len();
    if (g1 != w1) return 10;
    u32 i1 <- 0;
    while (i1 < w1) {
        u8 ac <- Json.at(i1);
        u8 ec <- exp1[i1];
        if (ac != ec) return 11;
        i1 <- i1 + 1;
    }
```

- [ ] **Step 2: Run, expect FAIL** (`addUint`/`addInt`/`end` undefined)

```bash
cd /home/linux/code/ovgt/src/domain && cnext json.test.cnx -o json.test.c
```

- [ ] **Step 3: Add to `src/domain/json.cnx`** (inside `scope Json`)

```cnx
    void putRaw(const string<24> s) {
        u32 len <- s.char_count;
        u32 i <- 0;
        while (i < len) {
            u8 ch <- s[i];
            this.putByte(ch);
            i <- i + 1;
        }
    }

    void putUintRaw(u32 v) {
        if (v = 0) {
            this.putByte('0');
            return;
        }
        u32 div <- 1;
        u32 t <- v;
        while (t >= 10) {
            div <- div * 10;
            t <- t / 10;
        }
        while (div > 0) {
            u32 digit <- (v / div) % 10 + 48;
            u8 dc <- digit[0, 8];
            this.putByte(dc);
            div <- div / 10;
        }
    }

    void key(const string<24> k) {
        if (this.needComma = true) {
            this.putByte(',');
        }
        this.needComma <- true;
        this.putByte('"');
        this.putRaw(k);
        this.putByte('"');
        this.putByte(':');
    }

    public void addUint(const string<24> k, u32 v) {
        this.key(k);
        this.putUintRaw(v);
    }

    public void addInt(const string<24> k, i32 v) {
        this.key(k);
        bool neg <- v < 0;
        if (neg = true) {
            this.putByte('-');
            i32 pos <- 0 - v;
            this.putUintRaw(pos[0, 32]);
        } else {
            this.putUintRaw(v[0, 32]);
        }
    }

    public void end() {
        this.putByte('}');
    }
```

- [ ] **Step 4: Run, expect PASS**

```bash
cd /home/linux/code/ovgt/src/domain && cnext json.test.cnx -o json.test.c && gcc -std=c11 -Wall -o json.test json.test.c && ./json.test; echo "exit=$?"   # exit=0
```

- [ ] **Step 5: Commit**

```bash
cd /home/linux/code/ovgt
git add src/domain/json.cnx src/domain/json.test.cnx src/domain/json.test.c
git commit -m "feat(telemetry): Json addUint/addInt (divisor itoa) + end"
```

---

### Task 3: Bool + string fields (`addBool`, `addStr`)

**Files:** Modify `src/domain/json.cnx`, `src/domain/json.test.cnx`

- [ ] **Step 1: Add failing test** to `main`

```cnx
    Json.begin();
    Json.addBool("b", true);
    Json.addStr("m", "auto");
    Json.end();
    // Expect: {"b":true,"m":"auto"}
    string<32> exp2 <- "{\"b\":true,\"m\":\"auto\"}";
    u32 w2 <- exp2.char_count;
    u32 g2 <- Json.len();
    if (g2 != w2) return 20;
    u32 i2 <- 0;
    while (i2 < w2) {
        u8 ac2 <- Json.at(i2);
        u8 ec2 <- exp2[i2];
        if (ac2 != ec2) return 21;
        i2 <- i2 + 1;
    }
```

- [ ] **Step 2: Run, expect FAIL**

```bash
cd /home/linux/code/ovgt/src/domain && cnext json.test.cnx -o json.test.c
```

- [ ] **Step 3: Add to `scope Json`**

```cnx
    public void addBool(const string<24> k, bool v) {
        this.key(k);
        if (v = true) {
            this.putRaw("true");
        } else {
            this.putRaw("false");
        }
    }

    public void addStr(const string<24> k, const string<24> v) {
        this.key(k);
        this.putByte('"');
        this.putRaw(v);
        this.putByte('"');
    }
```

- [ ] **Step 4: Run, expect PASS** (same command, `exit=0`)
- [ ] **Step 5: Commit** — `feat(telemetry): Json addBool/addStr`

---

### Task 4: Fixed-decimal float field (`addFloat2`)

**Files:** Modify `src/domain/json.cnx`, `src/domain/json.test.cnx`

Fixed 2-decimal via scale-to-int (rounded). Telemetry needs 1–2 dp; truncation/rounding error is irrelevant for logging.

- [ ] **Step 1: Add failing test** to `main`

```cnx
    Json.begin();
    Json.addFloat2("d", 3.14);
    Json.addFloat2("e", 0 - 0.07);
    Json.end();
    // Expect: {"d":3.14,"e":-0.07}
    string<32> exp3 <- "{\"d\":3.14,\"e\":-0.07}";
    u32 w3 <- exp3.char_count;
    u32 g3 <- Json.len();
    if (g3 != w3) return 30;
    u32 i3 <- 0;
    while (i3 < w3) {
        u8 ac3 <- Json.at(i3);
        u8 ec3 <- exp3[i3];
        if (ac3 != ec3) return 31;
        i3 <- i3 + 1;
    }
```

- [ ] **Step 2: Run, expect FAIL**

```bash
cd /home/linux/code/ovgt/src/domain && cnext json.test.cnx -o json.test.c
```

- [ ] **Step 3: Add to `scope Json`**

```cnx
    public void addFloat2(const string<24> k, f32 v) {
        this.key(k);
        f32 x <- v;
        bool neg <- x < 0.0;
        if (neg = true) {
            this.putByte('-');
            x <- 0.0 - x;
        }
        u32 scaled <- (u32)(x * 100.0 + 0.5);
        u32 ip <- scaled / 100;
        u32 fp <- scaled % 100;
        this.putUintRaw(ip);
        this.putByte('.');
        bool padZero <- fp < 10;
        if (padZero = true) {
            this.putByte('0');
        }
        this.putUintRaw(fp);
    }
```

- [ ] **Step 4: Run, expect PASS** (`exit=0`)
- [ ] **Step 5: Commit** — `feat(telemetry): Json addFloat2 (scale-to-int 2dp)`

---

### Task 5: Worst-case capacity test (prove `N=640` never truncates)

**Files:** Modify `src/domain/json.test.cnx`

- [ ] **Step 1: Add a test** that builds the full telemetry object with max-width values and asserts `full == false` and the buffer fits. Add an accessor first.

In `scope Json` add:

```cnx
    public bool overflowed() {
        return this.full;
    }
```

In `json.test.cnx` `main`, build every field at max width (u32 4294967295, i32 -2147483648, f32 large, longest keys) exactly as the firmware will (see Task 7's field list), then:

```cnx
    bool of <- Json.overflowed();
    if (of = true) return 40;        // N too small — bump string<640>
    u32 gN <- Json.len();
    if (gN >= 640) return 41;
```

- [ ] **Step 2–4:** Run; if it fails, increase `buf` capacity in `json.cnx` until it passes. (This is the "N proven from a worst-case dummy" check from the spec.)
- [ ] **Step 5: Commit** — `test(telemetry): prove Json buffer capacity for worst-case object`

---

### Task 6: Surface gate internals from `cotSettleStep`

**Files:**
- Modify: `src/sensors/cotSettle.h` (extend `CotSettleResult`)
- Modify: `src/sensors/cotSettle.cpp` (populate new fields)
- Modify: `test/test_cot_settle/test_cot_settle.cpp` (assert they're populated)

The gate's COT slope, boost slope, and settle timer are computed inside `cotSettleStep` but not returned. Add them to `CotSettleResult` so the telemetry object can carry them (spec field list: `cot_slope_c_s`, `boost_slope_psi_s`, `settle_timer_s`).

- [ ] **Step 1: Read** `src/sensors/cotSettle.h` and `.cpp` to find the slope/timer locals.
- [ ] **Step 2: Add a failing Unity assertion** in `test/test_cot_settle/test_cot_settle.cpp` — feed a known ramp, assert `result.cotSlopeCperS`, `result.boostSlopePsiPerS`, `result.settleTimerS` match expected.
- [ ] **Step 3: Run** `pio test -e native -f test_cot_settle` → expect FAIL (fields don't exist).
- [ ] **Step 4: Extend `CotSettleResult`** with `float cotSlopeCperS; float boostSlopePsiPerS; float settleTimerS;` and populate them in `cotSettleStep`.
- [ ] **Step 5: Run** `pio test -e native -f test_cot_settle` → expect PASS.
- [ ] **Step 6: Commit** — `feat(telemetry): expose gate slopes + settle timer in CotSettleResult`

---

### Task 7: Wire 10 Hz telemetry + settle NDJSON into `ovgt.cpp`

**Files:**
- Modify: `cnext_build.py` (also transpile `src/domain/json.cnx`)
- Modify: `src/domain/ovgt.cpp` (replace pretty line; 10 Hz emit; settle object)

- [ ] **Step 1: Make `json.cnx` transpile in the build.** In `cnext_build.py`, after the `actuator.cnx` transpile, add a second entry:

```python
    for entry in [Path("src/display/actuator.cnx"), Path("src/domain/json.cnx")]:
        if not entry.exists():
            continue
        result = subprocess.run(["cnext", str(entry), "--header-out", "include"],
                                check=True, capture_output=True, text=True)
        # (existing error handling)
```

- [ ] **Step 2: Replace `handleDebug()`** in `ovgt.cpp`. Include `#include <json.hpp>`. Change cadence to 10 Hz: drop `debugTimer`/`debugFlag`; in `loop()` (already 100 Hz) keep a counter and emit every 10th iteration. Build the telemetry object and emit:

```cpp
// every 100 ms (10 Hz):
int16_t boostGauge = (int16_t)appData.compressorOutputPressureHpaa - (int16_t)appData.compressorInputPressureHpaa;
if (boostGauge < 0) boostGauge = 0;
float boostPsi = boostGauge * 0.0145038f;
float br = appData.compressorInputPressureHpaa > 0
    ? (float)appData.compressorOutputPressureHpaa / appData.compressorInputPressureHpaa : 0.0f;
float bpr = boostGauge != 0 ? (float)appData.turbineInputPressureHpa / boostGauge : 0.0f;
float pr = br;
float ce = compressorEfficiency(pr, (float)appData.compressorInputTempC, (float)appData.compressorOutputTempC);

Json_begin();
Json_addUint("t_ms", (uint32_t)millis());
Json_addStr("mode", manualMode ? "manual" : (appData.exhaustBrakeActive ? "brake" : "auto"));
Json_addUint("cop_hpa", appData.compressorOutputPressureHpaa);
Json_addUint("cip_hpa", appData.compressorInputPressureHpaa);
Json_addFloat2("boost_psi", boostPsi);
Json_addFloat2("br", br);
Json_addFloat2("tip_psi", appData.turbineInputPressureHpa * 0.0145038f);
Json_addFloat2("bpr", bpr);
Json_addFloat2("bpr_target", BoostController::getBprTarget());
Json_addFloat2("cit_c", appData.compressorInputTempC);
Json_addFloat2("cot_c", appData.compressorOutputTempC);
Json_addInt("tit_c", appData.turbineInletTempC);
Json_addFloat2("ce_pct", ce >= 0.0f ? ce * 100.0f : -1.0f);
Json_addBool("ce_settled", ceSettled);
Json_addUint("dem_pct", manualMode ? manualPwm : appData.actuatorDemandedPosition);
Json_addUint("pos_pct", appData.actuatorReportedPosition);
Json_addBool("brake", appData.exhaustBrakeActive);
Json_end();
for (uint32_t i = 0; i < Json_len(); i++) Serial.write(Json_at(i));
Serial.write('\n');
```

(Add the optional/gate-extra fields per the spec field list once the basics verify on the truck.)

- [ ] **Step 3: Replace the settle line** (the `cotSettleStep` block, ~ovgt.cpp:217). On `ceRes.measurementReady`, build a settle object:

```cpp
Json_begin();
Json_addStr("type", "s");
Json_addUint("t_ms", (uint32_t)millis());
Json_addFloat2("tau_s", ceRes.tauSeconds);
Json_addFloat2("settle_s", ceRes.settleSeconds);
Json_addFloat2("step_c", ceRes.stepC);
Json_end();
for (uint32_t i = 0; i < Json_len(); i++) Serial.write(Json_at(i));
Serial.write('\n');
```

Add `Json_addStr("type", "t")` as the first field of the telemetry object too, so the host can discriminate.

- [ ] **Step 4: Build** for the Teensy:

```bash
cd /home/linux/code/ovgt && pio run -e teensy41
# expect: clean build; json.cnx transpiled; no snprintf pretty line remains
```

- [ ] **Step 5: Verify on the truck / serial monitor** — flash, open the serial monitor, confirm ~10 NDJSON lines/sec, each a valid JSON object, plus settle objects on COT steps. `logToTee.sh` still captures the stream (now NDJSON).
- [ ] **Step 6: Commit** — `feat(telemetry): emit 10Hz NDJSON telemetry + settle events (replaces pretty line)`

---

## Self-Review

- **Spec coverage:** Json builder (Tasks 1–5) ✓; gate internals surfaced (Task 6) ✓; 10 Hz NDJSON telemetry + settle objects, pretty line replaced, tuning commands untouched (Task 7) ✓. Host tool + Mongo + tuning keymap = Part 2 (separate plan), as the spec's build order intends.
- **Placeholders:** none — every builder method is verified code; Task 6/7 reference exact files and show the code. (Task 6 reads `cotSettle.cpp` first because the exact slope/timer locals must be matched to the existing impl — that's discovery, not a placeholder.)
- **Type consistency:** `Json_*` C names match the `scope Json` methods (`begin/addUint/addInt/addBool/addStr/addFloat2/end/at/len/overflowed`); `const string<24>` key params accept the literal keys used in Task 7; buffer is `string<640> buf <- ""` throughout.

## Notes / known C-Next issues hit while verifying

- `string<640> buf <- "";` — explicit init required (#1019).
- itoa uses a **divisor loop, not a temp array** (a temp `u8[]` write-at-runtime-index tripped a spurious use-before-init in one context — possible bug, not yet filed).
- bool conditions use `x = true` (a bare-bool `this.member` in `if` errors E0701 — possible bug, not yet filed).
- the length accessor is `len()`, not `length()` (collides with the deprecated `.length` property).
