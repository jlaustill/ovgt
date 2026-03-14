#include <Arduino.h>
#include "ovgt.h"
#include "display/actuator.h"
#include "AppData.h"
#include "display/lcdDisplay.h"
#include "sensors/adcSensors.h"
#include "control/boostController.h"

IntervalTimer debugTimer;

LcdDisplay lcdDisplay(0x27, 20, 4);

AppData appData;

volatile uint32_t ovgt::count;
volatile bool ovgt::debugFlag = false;

uint8_t ovgt::manualPwm = 0;
bool ovgt::manualMode = false;

void ovgt::handleDebugTimer() {
    debugFlag = true;
}

void ovgt::handleDebug() {
    if (!debugFlag) return;
    debugFlag = false;

    lcdDisplay.updateDisplay(count);

    bool pgGood = !digitalRead(PG_PIN);
    Serial.print("PG:");
    Serial.print(pgGood ? "OK" : "FAIL");
    Serial.print(" COP:");
    Serial.print(appData.boostPressureHpa);
    Serial.print("hPa Boost:");
    int16_t boostGauge = (int16_t)appData.boostPressureHpa - (int16_t)appData.ambientPressureGuessHpa;
    Serial.print(boostGauge);
    Serial.print("hPa TIPv:");
    Serial.print(appData.turbineInputVoltage, 3);
    Serial.print("V TIP:");
    Serial.print(appData.turbineInputPressureHpa);
    Serial.print("hPa BR:");
    if (boostGauge != 0) {
        Serial.print((float)appData.turbineInputPressureHpa / boostGauge, 2);
    } else {
        Serial.print("\xe2\x88\x9e");
    }
    Serial.print(" Amb:");
    Serial.print(appData.ambientPressureGuessHpa);
    Serial.print("hPa Dem:");
    if (manualMode) {
        Serial.print(manualPwm);
        Serial.print("%");
    } else {
        Serial.print(appData.actuatorDemandedPosition);
        Serial.print("%");
    }
    Serial.print(" Mode:");
    Serial.print(manualMode ? "MANUAL" : "AUTO");
    Serial.print(" Pos:");
    Serial.print(appData.actuatorReportedPosition);
    Serial.println("%");
    count = 0;
}


void ovgt::setup() {
    Serial.begin(115200);
    lcdDisplay.init();

    count = 0;
    appData.ambientPressureGuessHpa = 10000;
    pinMode(PG_PIN, INPUT);

    AdcSensors::Initialize();
    BoostController::Initialize();
    Actuator::Initialize();

    debugTimer.begin(handleDebugTimer, 1 * 1000 * 1000); // 1s

    Serial.println("Setup complete");
    Serial.println("Type a number 0-100 to set vane position %, or 'auto' for normal operation");
}

void ovgt::handleSerial() {
    static char buf[16];
    static uint8_t idx = 0;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (idx == 0) continue;
            buf[idx] = '\0';
            if (strcmp(buf, "auto") == 0) {
                manualMode = false;
                Serial.println("Switched to auto mode");
            } else {
                int val = atoi(buf);
                if (val >= 0 && val <= 100) {
                    manualPwm = (uint8_t)val;
                    manualMode = true;
                    Actuator::SetPosition(manualPwm);
                    Serial.print("Position set to ");
                    Serial.print(manualPwm);
                    Serial.println("%");
                } else {
                    Serial.println("Invalid: 0-100 or 'auto'");
                }
            }
            idx = 0;
        } else if (idx < 15) {
            buf[idx++] = c;
        }
    }
}

void ovgt::loop() {
    count++;
    handleSerial();
    handleDebug();

    // Safety: if 5V PSU power good is low, sensor data is unreliable
    if (digitalRead(PG_PIN)) {
        appData.actuatorDemandedPosition = SAFE_VANE_POSITION;
        Actuator::Loop();
        return;
    }

    AdcSensors::update();
    if (appData.boostPressureHpa > 500 && appData.boostPressureHpa < appData.ambientPressureGuessHpa) {
        appData.ambientPressureGuessHpa = appData.boostPressureHpa;
    }

    if (manualMode) {
        // Manual position hold — actuator stays at last set value, sensors keep running
        return;
    }

    BoostController::update();
    Actuator::Loop();
}