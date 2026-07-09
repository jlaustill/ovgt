# RIFE Hi-AT COT Thermistor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the slow MAX31856 K-type COT thermocouple with a fast RIFE Hi-AT NTC thermistor read on ADS2 channel 3, so compressor-efficiency (CE) is less lag-dominated.

**Architecture:** A new pure `cotThermistor` module converts the ADS divider voltage to °C (Steinhart-Hart) and is unit-tested on the host. `adcSensors` calls it on channel 3 with raw (un-smoothed) passthrough and flags each fresh COT sample. The main loop feeds `cotSettle` off that flag at the native ADS rate; `cotSettle`'s two count-based buffers grow to keep their time spans at that rate. The MAX31856 thermocouple path is deleted.

**Tech Stack:** C++ (Teensy 4.1 / PlatformIO / Arduino), Unity for native host tests, Adafruit ADS1X15.

## Global Constraints

- Firmware build: `pio run -e teensy41`. Native logic tests: `pio test -e native`.
- A newly tested `src/` file MUST be added to the `[env:native]` `build_src_filter` in `platformio.ini`.
- Solo private repo: commit directly to `main`, small frequent commits, no feature branches. Do not push unless asked.
- Naming: spell words out; no invented abbreviations (industry-standard ones like ADC/CAN/RPM/NTC are fine).
- Pull-up: **1.0 kΩ to 5 V, sensor to GND, on ADS2 (0x49) channel 3**. Divider inversion `R = 1000·V/(5−V)`.
- RIFE Steinhart-Hart coefficients: `A = 6.535185e-4`, `B = 2.345466e-4`, `C = 9.380459e-11` (`1/T(K) = A + B·lnR + C·lnR³`).
- Commit trailer on every commit: `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>`.

---

### Task 1: Pure `cotThermistor` conversion module

Pure voltage→°C conversion for the RIFE Hi-AT NTC, no Arduino/ADS dependencies, native-testable.

**Files:**
- Create: `src/sensors/cotThermistor.h`
- Create: `src/sensors/cotThermistor.cpp`
- Create: `test/test_cot_thermistor/test_cot_thermistor.cpp`
- Modify: `platformio.ini:53` (`[env:native]` `build_src_filter` — append `+<sensors/cotThermistor.cpp>`)

**Interfaces:**
- Produces: `bool cotThermistorReadC(float voltage, float *outTempC)` — returns `true` and writes `*outTempC` for a valid reading; returns `false` (leaves `*outTempC` untouched) when the divider is railed (`voltage >= 4.99f || voltage <= 0.01f`).

- [ ] **Step 1: Write the failing test**

Create `test/test_cot_thermistor/test_cot_thermistor.cpp`:

```cpp
#include <unity.h>
#include "sensors/cotThermistor.h"

void setUp(void) {}
void tearDown(void) {}

// Datasheet points (pull-up 1.0k, Vcc 5): R -> divider V -> °C, within 0.5 °C.
void test_datasheet_points(void) {
    float t;
    TEST_ASSERT_TRUE(cotThermistorReadC(4.3714f, &t));   // 6954 Ω, 200 °F
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 93.333f, t);
    TEST_ASSERT_TRUE(cotThermistorReadC(2.9236f, &t));   // 1408 Ω, 305 °F
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 151.667f, t);
    TEST_ASSERT_TRUE(cotThermistorReadC(0.8609f, &t));   // 208 Ω, 485 °F
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 251.667f, t);
}

// Railed samples (open/short) are rejected and leave the output untouched.
void test_rail_reject_holds_last(void) {
    float t = -999.0f;
    TEST_ASSERT_FALSE(cotThermistorReadC(4.99f, &t));    // open (near 5 V)
    TEST_ASSERT_EQUAL_FLOAT(-999.0f, t);
    TEST_ASSERT_FALSE(cotThermistorReadC(0.01f, &t));    // short (near 0 V)
    TEST_ASSERT_EQUAL_FLOAT(-999.0f, t);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_datasheet_points);
    RUN_TEST(test_rail_reject_holds_last);
    return UNITY_END();
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_cot_thermistor`
Expected: FAIL — `fatal error: sensors/cotThermistor.h: No such file or directory`.

- [ ] **Step 3: Write the module**

Create `src/sensors/cotThermistor.h`:

```cpp
#ifndef cotThermistor_h
#define cotThermistor_h

// Convert the ADS divider voltage on the COT channel to °C for the RIFE Hi-AT
// NTC thermistor (1.0 kΩ pull-up to 5 V, sensor to GND). Returns false when the
// divider is railed (open/short) so the caller can hold its last good value.
bool cotThermistorReadC(float voltage, float *outTempC);

#endif
```

Create `src/sensors/cotThermistor.cpp`:

```cpp
#include "cotThermistor.h"
#include <math.h>

static const float PULLUP_OHMS = 1000.0f;   // pull-up to 5 V, RIFE NTC to GND
static const float SUPPLY_VOLTS = 5.0f;
static const float RAIL_HIGH_VOLTS = 4.99f; // >= this reads as open circuit
static const float RAIL_LOW_VOLTS = 0.01f;  // <= this reads as short circuit

// RIFE Hi-AT Steinhart-Hart: 1/T(K) = A + B·lnR + C·lnR³ (0.11 °C fit over table).
static const float STEINHART_A = 6.535185e-4f;
static const float STEINHART_B = 2.345466e-4f;
static const float STEINHART_C = 9.380459e-11f;

bool cotThermistorReadC(float voltage, float *outTempC) {
    if (voltage >= RAIL_HIGH_VOLTS || voltage <= RAIL_LOW_VOLTS) {
        return false;
    }
    float resistance = PULLUP_OHMS * voltage / (SUPPLY_VOLTS - voltage);
    float lnR = logf(resistance);
    float invT = STEINHART_A + STEINHART_B * lnR + STEINHART_C * lnR * lnR * lnR;
    *outTempC = (1.0f / invT) - 273.15f;
    return true;
}
```

- [ ] **Step 4: Register the module for the native build**

Modify `platformio.ini` line 53 — append `+<sensors/cotThermistor.cpp>` to the `[env:native]` `build_src_filter` (keep all existing entries):

```
build_src_filter = -<*> +<control/exhaustBrakeLogic.cpp> +<control/boostBprLogic.cpp> +<sensors/j1939Decode.cpp> +<sensors/j1939Health.cpp> +<sensors/j1939Encode.cpp> +<sensors/compressorEfficiency.cpp> +<sensors/cotSettle.cpp> +<sensors/cotThermistor.cpp> +<sensors/mcuHealthFrame.cpp> +<domain/json.c> +<domain/systemHealthLogic.c>
```

- [ ] **Step 5: Run test to verify it passes**

Run: `pio test -e native -f test_cot_thermistor`
Expected: PASS (2 tests).

- [ ] **Step 6: Commit**

```bash
git add src/sensors/cotThermistor.h src/sensors/cotThermistor.cpp test/test_cot_thermistor/test_cot_thermistor.cpp platformio.ini
git commit -m "feat(cot): pure RIFE Hi-AT thermistor voltage->C conversion module

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 2: Grow `cotSettle` buffers for the native ADS feed rate

`cotSettle`'s two count-based arrays were sized for the thermocouple's ~10 Hz feed. The ADS round-robin delivers COT at ~200 Hz; grow the arrays so their *time* spans survive. Everything else in `cotSettle` is time-based and self-adjusts.

**Files:**
- Modify: `src/sensors/cotSettle.h:28` (`COT_SETTLE_BUFFER`) and `src/sensors/cotSettle.h:32` (`COT_SLOPE_WINDOW`)
- Test: `test/test_cot_settle/test_cot_settle.cpp` (add one high-rate case)

**Interfaces:**
- Consumes: nothing new.
- Produces: unchanged `cotSettleStep()` API; larger buffer capacity only.

- [ ] **Step 1: Write the failing test**

Add to `test/test_cot_settle/test_cot_settle.cpp` (append the function and its `RUN_TEST` line inside `main`):

```cpp
// At the native ADS rate (~200 Hz, dt = 0.005 s) a sustained-flat COT + boost must
// still settle: the 0.5 s windowed-slope flatness needs the slope ring to hold a
// full 0.5 s (>= 100 samples), which only passes with the grown COT_SLOPE_WINDOW.
void test_settled_flag_at_ads_rate(void) {
    CotSettleConfig cfg = makeConfig();
    CotSettleState st; cotSettleInit(st);
    cotSettleStep(st, cfg, 50.0f, 5.0f, 0.005f);  // seed
    CotSettleResult r = {false, false, 0, 0, 0};
    for (int k = 0; k < 500; k++) r = cotSettleStep(st, cfg, 50.0f, 5.0f, 0.005f);
    TEST_ASSERT_TRUE(r.settled);  // 500*0.005 = 2.5 s >= 2.0 s
    TEST_ASSERT_TRUE(COT_SLOPE_WINDOW >= 100);  // ring must span 0.5 s at 200 Hz
}
```

Add inside `main`, next to the other `RUN_TEST(...)` lines:

```cpp
    RUN_TEST(test_settled_flag_at_ads_rate);
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_cot_settle`
Expected: FAIL on `TEST_ASSERT_TRUE(COT_SLOPE_WINDOW >= 100)` (current value is 16).

- [ ] **Step 3: Grow the buffers**

Modify `src/sensors/cotSettle.h`. Replace line 27-28:

```cpp
// Max samples buffered during one measurement. ~2048 @ ~200 Hz covers ~10 s.
static const uint16_t COT_SETTLE_BUFFER = 2048;
```

Replace line 30-32:

```cpp
// Ring buffer for the settled-flag's windowed flatness slope. ~128 @ ~200 Hz covers
// ~0.64 s, comfortably more than any sane slopeWindowSeconds (stays in uint8_t ring).
static const uint8_t COT_SLOPE_WINDOW = 128;
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pio test -e native -f test_cot_settle`
Expected: PASS (all cases, including the two pre-existing settled-flag tests and the new one).

- [ ] **Step 5: Commit**

```bash
git add src/sensors/cotSettle.h test/test_cot_settle/test_cot_settle.cpp
git commit -m "feat(cot): size cotSettle buffers for the ~200 Hz ADS feed

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 3: Read COT on ADS2 channel 3 (raw passthrough + fresh-sample flag)

Repurpose ADS2 channel 3 from lift-pump pressure to the RIFE COT thermistor: raw (un-smoothed) reading via `cotThermistorReadC`, holding the last good value on rail-reject, and a flag the main loop can poll for each fresh COT sample.

**Files:**
- Modify: `src/sensors/adcSensors.h` (declare `cotSampleReady()`, `cotDtSeconds`/`clearCotSample`)
- Modify: `src/sensors/adcSensors.cpp` (include `cotThermistor.h`; ch3 decode; raw passthrough for ch3; fresh-sample flag)

**Interfaces:**
- Consumes: `bool cotThermistorReadC(float, float*)` from Task 1.
- Produces:
  - `static bool AdcSensors::cotSampleReady()` — true when a fresh channel-3 COT sample was stored since the last `clearCotSample()`.
  - `static void AdcSensors::clearCotSample()` — clears the fresh flag.
  - `appData.compressorOutputTempC` now sourced from ADS2 ch3.

- [ ] **Step 1: Add the fresh-sample accessors to the header**

Modify `src/sensors/adcSensors.h`. Add to the `public:` section (after `static void update();`):

```cpp
        static bool cotSampleReady();
        static void clearCotSample();
```

Add to the `private:` section (near the ADS2 statics):

```cpp
        static bool cotFresh;
```

- [ ] **Step 2: Define the flag and accessors in the .cpp**

Modify `src/sensors/adcSensors.cpp`. Add the include near the top (after `#include "adcSensors.h"`):

```cpp
#include "cotThermistor.h"
```

Add the static definition near the other `AdcSensors::` static definitions (around line 64):

```cpp
bool AdcSensors::cotFresh = false;
```

Add the accessor definitions (e.g. after `AdcSensors::update()`):

```cpp
bool AdcSensors::cotSampleReady() { return cotFresh; }
void AdcSensors::clearCotSample() { cotFresh = false; }
```

- [ ] **Step 3: Raw passthrough for channel 3 in `updateAds2`**

Modify `src/sensors/adcSensors.cpp` `updateAds2()`. Replace the EMA block (lines ~141-147) so channel 3 passes the raw voltage while channels 0-2 keep their EMA:

```cpp
        int16_t raw2 = ads2.getLastConversionResults();
        float voltage2 = ads2.computeVolts(raw2);
        float value2;
        if (currentChannel2 == 3) {
            value2 = voltage2;                 // COT: raw, no smoothing (preserve response)
        } else {
            if (!emaInitialized2[currentChannel2]) {
                ema2[currentChannel2] = voltage2;
                emaInitialized2[currentChannel2] = true;
            } else {
                ema2[currentChannel2] += EMA_ALPHA * (voltage2 - ema2[currentChannel2]);
            }
            value2 = ema2[currentChannel2];
        }
        processResult2(currentChannel2, value2);
```

- [ ] **Step 4: Replace the channel-3 decode**

Modify `src/sensors/adcSensors.cpp` `processResult2()`. Replace `case 3` (the lift-pump block, lines ~233-240) with:

```cpp
        case 3: {
            // Compressor outlet temp: RIFE Hi-AT NTC, 1.0 kΩ pull-up to 5 V, NTC to GND.
            // Raw sample (no EMA); hold last good on a railed (open/short) reading.
            float tempC;
            if (cotThermistorReadC(voltage, &tempC)) {
                appData.compressorOutputTempC = tempC;
                cotFresh = true;
            }
            break;
        }
```

- [ ] **Step 5: Build the firmware to verify it compiles**

Run: `pio run -e teensy41`
Expected: build succeeds. (`liftPumpPressureHpa` is still declared in `AppData.h`; it becomes unreferenced but the field is removed in Task 4.)

- [ ] **Step 6: Commit**

```bash
git add src/sensors/adcSensors.h src/sensors/adcSensors.cpp
git commit -m "feat(cot): read RIFE COT thermistor on ADS2 ch3, raw + fresh flag

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 4: Feed cotSettle from the ADS COT sample; remove the thermocouple

Move the `cotSettle` feed off the deleted thermocouple and onto the ADS fresh-sample flag, then delete the MAX31856 path and the orphaned lift-pump field.

**Files:**
- Modify: `src/domain/ovgt.cpp` (drop the `CotSensor` include/init/loop-block; feed `cotSettleStep` from `AdcSensors::cotSampleReady()`; drop pin 37 from the CS-high loop)
- Delete: `src/sensors/cotSensor.cpp`, `src/sensors/cotSensor.h`
- Modify: `platformio.ini:27` (remove the MAX31856 lib dependency)
- Modify: `include/AppData.h:25` (remove `liftPumpPressureHpa`)

**Interfaces:**
- Consumes: `AdcSensors::cotSampleReady()` / `clearCotSample()` (Task 3); `cotSettleStep()` (Task 2).
- Produces: no new interfaces.

- [ ] **Step 1: Remove the CotSensor include and init**

Modify `src/domain/ovgt.cpp`. Delete the include line `#include "sensors/cotSensor.h"` (line ~9). In `ovgt::setup()`, delete these two lines (~265-266):

```cpp
    CotSensor::Initialize();
    SystemHealth_feed();
```

In the SPI-CS-high loop in `setup()` (line ~256), remove `37` from the pin list (it was the MAX31856 chip-select):

```cpp
    for (uint8_t pin : {3, 4, 5, 10, 24, 25, 26, 35, 38, 39, 40}) {
```

- [ ] **Step 2: Re-point the cotSettle feed in `ovgt::loop`**

Modify `src/domain/ovgt.cpp` `ovgt::loop()`. Replace the `if (CotSensor::update()) { ... }` block (lines ~288-312) with a version driven by the ADS fresh-sample flag:

```cpp
    if (AdcSensors::cotSampleReady()) {
        AdcSensors::clearCotSample();
        float cotDt = cotSampleDt / 1000000.0f;
        cotSampleDt = 0;
        int16_t boostHpa = (int16_t)appData.compressorOutputPressureHpaa
                         - (int16_t)appData.compressorInputPressureHpaa;
        if (boostHpa < 0) boostHpa = 0;
        CotSettleResult ceRes = cotSettleStep(cotSettleState, cotSettleCfg,
                                              appData.compressorOutputTempC,
                                              boostHpa * 0.0145038f, cotDt);
        ceSettled = ceRes.settled;
        if (ceRes.measurementReady) {
            Json_begin();
            Json_addStr("type", "s");
            Json_addUint("t_ms", (uint32_t)millis());
            Json_addFloat2("tau_s", ceRes.tauSeconds);
            Json_addFloat2("settle_s", ceRes.settleSeconds);
            Json_addFloat2("step_c", ceRes.stepC);
            Json_addFloat2("cot_slope_c_s", ceRes.cotSlopeCperS);
            Json_addFloat2("boost_slope_psi_s", ceRes.boostSlopePsiPerS);
            Json_addFloat2("settle_timer_s", ceRes.settleTimerS);
            Json_end();
            for (uint32_t i = 0; i < Json_len(); i++) Serial.write(Json_at(i));
            Serial.write('\n');
        }
    }
```

- [ ] **Step 3: Delete the thermocouple module**

```bash
git rm src/sensors/cotSensor.cpp src/sensors/cotSensor.h
```

- [ ] **Step 4: Remove the MAX31856 library dependency**

Modify `platformio.ini`. Delete line 27:

```
    adafruit/Adafruit MAX31856 library
```

- [ ] **Step 5: Remove the orphaned lift-pump field**

Modify `include/AppData.h`. Delete line 25:

```cpp
    uint16_t liftPumpPressureHpa;
```

- [ ] **Step 6: Build the firmware to verify it compiles**

Run: `pio run -e teensy41`
Expected: build succeeds with no reference to `CotSensor`, `Adafruit_MAX31856`, or `liftPumpPressureHpa`.

- [ ] **Step 7: Run the native suite to confirm nothing regressed**

Run: `pio test -e native`
Expected: PASS (includes `test_cot_thermistor` and `test_cot_settle`).

- [ ] **Step 8: Commit**

```bash
git add src/domain/ovgt.cpp platformio.ini include/AppData.h
git commit -m "feat(cot): drive cotSettle from ADS COT, remove MAX31856 thermocouple

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## On-hardware verification (after Task 4, not an automated gate)

Wire the RIFE sensor (1.0 kΩ pull-up to 5 V, sensor to GND, ADS2 ch3), flash, and
capture serial without resetting the board:

```
stty -F /dev/ttyACM0 115200 raw -echo
timeout 20 cat /dev/ttyACM0
```

Confirm on the `"t"` telemetry line: `cot_c` tracks a plausible outlet temp and
*responds fast* to a throttle blip (the whole point of the swap). Measure the true
channel-3 update rate; if it differs materially from ~200 Hz, adjust `COT_SLOPE_WINDOW`
(keep ≥ 100 to span 0.5 s, ≤ 128 to stay in the `uint8_t` ring) and `COT_SETTLE_BUFFER`
in `src/sensors/cotSettle.h`.

---

## Self-Review

- **Spec coverage:** §1 extracted module → Task 1. §2 ADS2 ch3 decode + raw passthrough → Task 3. §3 cotSettle fed at native rate + resized buffers → Tasks 2 & 4. §4 remove MAX31856 (files, lib, pin 37) → Task 4. §5 remove `liftPumpPressureHpa` → Task 4. Unchanged consumers (CE calc, `cot_c`, J1939 broadcast) require no edits — confirmed they read `appData.compressorOutputTempC` directly. Testing (datasheet points, rail-reject, high-rate settle) → Tasks 1 & 2. On-hardware verification → dedicated section.
- **Placeholder scan:** none — every code step shows full code; exact vectors and commands given.
- **Type consistency:** `cotThermistorReadC(float, float*)->bool` defined in Task 1, consumed in Task 3. `cotSampleReady()`/`clearCotSample()` defined in Task 3, consumed in Task 4. `COT_SLOPE_WINDOW`/`COT_SETTLE_BUFFER` grown in Task 2, relied on at runtime in Task 4. Consistent.
