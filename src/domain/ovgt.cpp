#include <Arduino.h>
#include "ovgt.h"
#include "display/actuator.h"
#include "AppData.h"
#include "sensors/adcSensors.h"
#include "sensors/titSensor.h"
#include "sensors/cotSensor.h"
#include "sensors/citSensor.h"
#include "sensors/totSensor.h"
#include "sensors/j1939.h"
#include "control/boostController.h"

IntervalTimer debugTimer;
elapsedMillis loopElapsed;
elapsedMicros adcElapsed;

AppData appData;

volatile uint32_t ovgt::count;
volatile bool ovgt::debugFlag = false;

uint8_t ovgt::manualPwm = 0;
bool ovgt::manualMode = false;
uint32_t ovgt::cyclesAdc = 0;
uint32_t ovgt::cyclesBoost = 0;
uint32_t ovgt::cyclesActuator = 0;
uint32_t ovgt::cyclesDebug = 0;

void ovgt::handleDebugTimer() {
    debugFlag = true;
}

void ovgt::handleDebug() {
    if (!debugFlag) return;
    debugFlag = false;

    int16_t boostGauge = (int16_t)appData.boostPressureHpa - (int16_t)appData.ambientPressureGuessHpa;

    char brBuf[16];
    if (boostGauge != 0) {
        snprintf(brBuf, sizeof(brBuf), "%.2f", (float)appData.turbineInputPressureHpa / boostGauge);
    } else {
        snprintf(brBuf, sizeof(brBuf), "\xe2\x88\x9e");
    }

    char buf[128];
    snprintf(buf, sizeof(buf), "Boost:%.1fpsi BPR:%s Dem:%u%% Pos:%u%% TIPv:%.3f COT:%dC CIT:%dC TIT:%dC TOT:%dC",
        (double)(boostGauge * 0.0145038f),
        brBuf,
        manualMode ? manualPwm : appData.actuatorDemandedPosition,
        appData.actuatorReportedPosition,
        (double)appData.turbineInputVoltage,
        appData.compressorOutputTempC,
        appData.compressorInputTempC,
        appData.turbineInletTempC,
        appData.turbineOutletTempC);
    Serial.println(buf);

    count = 0;
    cyclesAdc = 0;
    cyclesBoost = 0;
    cyclesActuator = 0;
    cyclesDebug = 0;
}


void ovgt::setup() {
    Serial.begin(115200);

    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

    count = 0;
    appData.ambientPressureGuessHpa = 10000;
    pinMode(PG_PIN, INPUT);

    // AdcSensors::Initialize();
    TitSensor::Initialize();
    // CotSensor::Initialize();
    // CitSensor::Initialize();
    // TotSensor::Initialize();
    // BoostController::Initialize();
    Actuator::Initialize();
    J1939::Initialize();

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
                    appData.actuatorDemandedPosition = manualPwm;
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
    handleSerial();

    // AdcSensors::update();
    TitSensor::update();
    // CotSensor::update();
    // CitSensor::update();
    // TotSensor::update();

    if (loopElapsed < 10) return;
    loopElapsed = 0;

    uint32_t t0, t1;
    count++;

    t0 = ARM_DWT_CYCCNT;
    handleDebug();
    t1 = ARM_DWT_CYCCNT;
    cyclesDebug += t1 - t0;

    appData.pgFault = digitalRead(PG_PIN);

    if (appData.boostPressureHpa > 500 && appData.boostPressureHpa < appData.ambientPressureGuessHpa) {
        appData.ambientPressureGuessHpa = appData.boostPressureHpa;
    }

    // BoostController::update();

    J1939::Loop();

    t0 = ARM_DWT_CYCCNT;
    Actuator::Loop();
    t1 = ARM_DWT_CYCCNT;
    cyclesActuator += t1 - t0;
}