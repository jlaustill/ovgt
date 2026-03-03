# Boost Control Loop Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add closed-loop VGT vane control driven by a 10 bar MAP sensor via ADS1115 ADC, with a tunable pressure-to-position lookup table.

**Architecture:** Three modules with single responsibilities — BoostSensor reads hardware and outputs hPa, BoostController maps hPa to vane position % via lookup table, Actuator takes a % and drives PWM. All communicate through the shared AppData struct.

**Tech Stack:** C++ / Arduino framework / PlatformIO / Teensy 4.1 / Adafruit ADS1X15 library

**Design doc:** `docs/plans/2026-03-03-boost-control-loop-design.md`

---

### Task 1: Add dependencies and extend AppData

**Files:**
- Modify: `platformio.ini:15-19`
- Modify: `include/AppData.h:6-12`

**Step 1: Add ADS1X15 library to platformio.ini**

Add Adafruit BusIO and ADS1X15 back to lib_deps:

```ini
lib_deps =
	https://github.com/tonton81/FlexCAN_T4
    https://github.com/adafruit/Adafruit_BusIO
    https://github.com/adafruit/Adafruit_ADS1X15
    Wire
    SPI
    LiquidCrystal_I2C
```

**Step 2: Add boostPressureHpa field to AppData**

```cpp
struct AppData {
    uint8_t actuatorTemp;
    uint8_t actuatorDemandedPosition;
    uint8_t actuatorReportedPosition;
    uint16_t actuatorMotorLoad;
    uint8_t actuatorStatus;
    uint16_t boostPressureHpa;
};
```

**Step 3: Build to verify**

Run: `pio run`
Expected: SUCCESS (no code uses the new field yet, just confirming libs resolve)

**Step 4: Commit**

```bash
git add platformio.ini include/AppData.h
git commit -m "Add ADS1X15 dependency and boostPressureHpa to AppData"
```

---

### Task 2: Create BoostSensor

**Files:**
- Create: `src/sensors/boostSensor.h`
- Create: `src/sensors/boostSensor.cpp`

**Step 1: Create boostSensor.h**

```cpp
#ifndef BoostSensor_h
#define BoostSensor_h

#include "AppData.h"

class BoostSensor {
    public:
        static void Initialize(AppData *appData, uint8_t channel = 0);
        static void read();
    private:
        static AppData *appData;
        static uint8_t channel;
};

#endif
```

**Step 2: Create boostSensor.cpp**

```cpp
#include <Arduino.h>
#include "boostSensor.h"
#include <Adafruit_ADS1X15.h>

static Adafruit_ADS1115 ads;

AppData *BoostSensor::appData;
uint8_t BoostSensor::channel = 0;

void BoostSensor::Initialize(AppData *currentData, uint8_t ch) {
    appData = currentData;
    channel = ch;
    ads.setGain(GAIN_TWOTHIRDS); // +/-6.144V range for 5V sensor
    ads.begin();
    Serial.println("BoostSensor initialized");
}

void BoostSensor::read() {
    int16_t raw = ads.readADC_SingleEnded(channel);
    float voltage = ads.computeVolts(raw);

    // 10 bar MAP sensor: 0.5V = 0 hPa, 4.5V = 10000 hPa
    // hPa = (voltage - 0.5) * 2500
    float hPa = (voltage - 0.5f) * 2500.0f;

    // Clamp to valid range
    if (hPa < 0.0f) hPa = 0.0f;
    if (hPa > 10000.0f) hPa = 10000.0f;

    appData->boostPressureHpa = (uint16_t)hPa;
}
```

**Step 3: Build to verify**

Run: `pio run`
Expected: SUCCESS (BoostSensor compiles but isn't called yet)

**Step 4: Commit**

```bash
git add src/sensors/boostSensor.h src/sensors/boostSensor.cpp
git commit -m "Add BoostSensor module for ADS1115 pressure reading"
```

---

### Task 3: Create BoostController

**Files:**
- Create: `src/control/boostController.h`
- Create: `src/control/boostController.cpp`

**Step 1: Create boostController.h**

```cpp
#ifndef BoostController_h
#define BoostController_h

#include "AppData.h"

struct PressureMapEntry {
    uint16_t pressureHpa;
    uint8_t positionPercent;
};

class BoostController {
    public:
        static void Initialize(AppData *appData);
        static void update();
    private:
        static AppData *appData;
        static const PressureMapEntry pressureMap[];
        static const uint8_t pressureMapSize;
        static uint8_t interpolate(uint16_t pressureHpa);
};

#endif
```

**Step 2: Create boostController.cpp**

This is a meaningful implementation choice — the lookup table and interpolation logic. The user should write the `interpolate` function body and define the lookup table entries.

```cpp
#include "boostController.h"

AppData *BoostController::appData;

// Tunable lookup table: pressure (absolute hPa) -> vane position (0-100%)
// Edit these values and recompile to tune on the truck.
const PressureMapEntry BoostController::pressureMap[] = {
    {1000, 21},
    {1500, 25},
    {2000, 35},
    {3000, 50}
};
const uint8_t BoostController::pressureMapSize = sizeof(pressureMap) / sizeof(pressureMap[0]);

void BoostController::Initialize(AppData *currentData) {
    appData = currentData;
    Serial.println("BoostController initialized");
}

void BoostController::update() {
    uint8_t position = interpolate(appData->boostPressureHpa);
    appData->actuatorDemandedPosition = position;
}

uint8_t BoostController::interpolate(uint16_t pressureHpa) {
    // TODO: User implements interpolation logic
    // Clamp below first entry, clamp above last entry,
    // linearly interpolate between entries.
}
```

**Step 3: Build to verify**

Run: `pio run`
Expected: SUCCESS (BoostController compiles but isn't called yet)

**Step 4: Commit**

```bash
git add src/control/boostController.h src/control/boostController.cpp
git commit -m "Add BoostController with tunable pressure-to-position lookup table"
```

---

### Task 4: Modify Actuator to read from AppData

**Files:**
- Modify: `src/display/actuator.cpp:7-9,63,66-90`

**Step 1: Remove hardcoded position variables and climbing logic**

Remove the global variables `p`, `maxp`, `minp` (lines 7-9). Simplify `Actuator::Loop()` to read position from `appData->actuatorDemandedPosition` instead.

Replace `actuator.cpp` `Loop()` and globals with:

```cpp
// Remove these globals:
// uint8_t p = 50;
// uint8_t maxp = 91;
// uint8_t minp = 7;

// In Initialize(), remove: SetPosition(p);
// Replace with: SetPosition(appData->actuatorDemandedPosition);

// Replace entire Loop() with:
void Actuator::Loop() {
    static uint32_t timeout = millis();
    if (millis() - timeout > 100) {
        SetPosition(appData->actuatorDemandedPosition);
        timeout = millis();
    }
}
```

Also update `SetPosition` to use literal bounds instead of the removed globals:

```cpp
void Actuator::SetPosition(uint8_t position) {
    if (position > 100) {
        position = 100;
    }
    int ap = map(position, 0, 100, 7, 91);
    ap = map(ap, 0, 100, 7, 247);
    analogWrite(actuatorPin, ap);
}
```

**Step 2: Build to verify**

Run: `pio run`
Expected: SUCCESS

**Step 3: Commit**

```bash
git add src/display/actuator.cpp
git commit -m "Simplify Actuator to read demanded position from AppData"
```

---

### Task 5: Wire everything into the main loop

**Files:**
- Modify: `src/domain/ovgt.cpp:1-6,27-38,40-44`
- Modify: `src/domain/ovgt.h`

**Step 1: Add includes and initialization calls in ovgt.cpp**

Add includes for BoostSensor and BoostController. Add their `Initialize()` calls in `setup()`. Add their update calls in `loop()`.

Updated `ovgt.cpp`:

```cpp
#include <Arduino.h>
#include "ovgt.h"
#include "display/actuator.h"
#include "AppData.h"
#include "display/lcdDisplay.h"
#include "sensors/boostSensor.h"
#include "control/boostController.h"

IntervalTimer debugTimer;

LcdDisplay lcdDisplay(0x27, 20, 4);

uint32_t ovgt::count;
AppData ovgt::appData;

void ovgt::handleDebugTimer() {
    lcdDisplay.updateDisplay(count);

    Serial.print("Loop count/Sec: ");
    Serial.print(count);
    Serial.print(" Boost: ");
    Serial.print(appData.boostPressureHpa);
    Serial.println(" hPa");
    count = 0;
}

void ovgt::setup() {
    Serial.begin(115200);
    lcdDisplay.init(&ovgt::appData);

    count = 0;

    BoostSensor::Initialize(&ovgt::appData);
    BoostController::Initialize(&ovgt::appData);
    Actuator::Initialize(&ovgt::appData);

    debugTimer.begin(handleDebugTimer, 1 * 1000 * 1000); // 1s

    Serial.println("Setup complete");
}

void ovgt::loop() {
    count++;

    BoostSensor::read();
    BoostController::update();
    Actuator::Loop();
}
```

**Step 2: Build to verify**

Run: `pio run`
Expected: SUCCESS

**Step 3: Commit**

```bash
git add src/domain/ovgt.cpp src/domain/ovgt.h
git commit -m "Wire BoostSensor and BoostController into main loop"
```

---

### Task 6: Update LCD to show boost pressure

**Files:**
- Modify: `src/display/lcdDisplay.cpp:15-30`

**Step 1: Add boost pressure to LCD output**

The LCD has 4 lines of 20 characters. Rearrange to include boost:
- Line 0: Boost pressure in hPa
- Line 1: Temperature + Motor Load
- Line 2: Demanded position
- Line 3: Reported position

```cpp
void LcdDisplay::updateDisplay(uint32_t loopCount) {
    lcd.setCursor(0, 0);
    snprintf(buffer, 21, "Boost %5u hPa", appData->boostPressureHpa);
    lcd.print(buffer);

    lcd.setCursor(0, 1);
    snprintf(buffer, 21, "%2dC Load %5u S:%1d", appData->actuatorTemp, appData->actuatorMotorLoad, appData->actuatorStatus);
    lcd.print(buffer);

    lcd.setCursor(0, 2);
    snprintf(buffer, 21, "Demanded  %3d%%", appData->actuatorDemandedPosition);
    lcd.print(buffer);

    lcd.setCursor(0, 3);
    snprintf(buffer, 21, "Reported  %3d%%", appData->actuatorReportedPosition);
    lcd.print(buffer);
}
```

**Step 2: Build to verify**

Run: `pio run`
Expected: SUCCESS

**Step 3: Commit**

```bash
git add src/display/lcdDisplay.cpp
git commit -m "Update LCD to display boost pressure"
```

---

### Task 7: Upload, verify on hardware, final commit

**Step 1: Upload to Teensy**

Run: `pio run --target upload`

**Step 2: Verify on serial monitor**

Open serial: `pio device monitor -b 115200`

Expected output:
- `BoostSensor initialized`
- `BoostController initialized`
- `Setup complete`
- `Loop count/Sec: XXXX Boost: YYYY hPa` every second
- Boost value should read ~1013 hPa at sea level with no boost applied
- CAN messages should still appear when actuator is connected

**Step 3: Verify LCD**

LCD should show:
```
Boost  1013 hPa
28C Load   123 S:0
Demanded   21%
Reported   21%
```

**Step 4: Final commit and push**

```bash
git push
```
