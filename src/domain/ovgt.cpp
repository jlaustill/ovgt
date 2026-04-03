#include <Arduino.h>
#include "ovgt.h"
#include "display/actuator.h"
#include "AppData.h"
#include "sensors/adcSensors.h"
#include "control/boostController.h"

IntervalTimer debugTimer;
elapsedMillis loopElapsed;

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

    char buf[160];
    snprintf(buf, sizeof(buf),
        "PG:%s COP:%uhPa Boost:%dhPa(%.1fpsi) TIPv:%.3fV TIP:%uhPa BR:%s Amb:%uhPa Dem:%u%% Mode:%s Pos:%u%%",
        appData.pgFault ? "FAULT" : "OK",
        appData.boostPressureHpa,
        boostGauge,
        (double)(boostGauge * 0.0145038f),
        (double)appData.turbineInputVoltage,
        appData.turbineInputPressureHpa,
        brBuf,
        appData.ambientPressureGuessHpa,
        manualMode ? manualPwm : appData.actuatorDemandedPosition,
        BoostController::benchMode ? "BENCH" : (manualMode ? "MANUAL" : "AUTO"),
        appData.actuatorReportedPosition);
    Serial.println(buf);

    char pidBuf[80];
    snprintf(pidBuf, sizeof(pidBuf), "  PID:%s BPR:%.2f->%.2f Out:%u%%",
        appData.pidActive ? "ACTIVE" : "OFF",
        (double)appData.currentBpr,
        (double)appData.targetBpr,
        appData.actuatorDemandedPosition);
    Serial.println(pidBuf);

    if (count > 0) {
        uint32_t totalUs = (cyclesAdc + cyclesBoost + cyclesActuator + cyclesDebug) / 600;
        snprintf(buf, sizeof(buf), "  loops:%lu us/s ADC:%lu Boost:%lu Act:%lu Dbg:%lu total:%lu (%.1f%%)",
            count,
            cyclesAdc / 600,
            cyclesBoost / 600,
            cyclesActuator / 600,
            cyclesDebug / 600,
            totalUs,
            (double)totalUs / 10000.0);
        Serial.println(buf);
    }
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

    AdcSensors::Initialize();
    BoostController::Initialize();
    Actuator::Initialize();

    debugTimer.begin(handleDebugTimer, 1 * 1000 * 1000); // 1s

    Serial.println("Setup complete");
    Serial.println("Commands: 0-100 (manual), 'auto', 'bench', 'bpr <val>', 'pid', 'pid <Kp> <Ki> <Kd>'");
}

void ovgt::handleSerial() {
    static char buf[32];
    static uint8_t idx = 0;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (idx == 0) continue;
            buf[idx] = '\0';
            if (strcmp(buf, "auto") == 0) {
                manualMode = false;
                BoostController::benchMode = false;
                Serial.println("Switched to auto mode");
            } else if (strcmp(buf, "bench") == 0) {
                manualMode = false;
                BoostController::benchMode = true;
                BoostController::benchTarget = 51.0f;
                Serial.println("Bench mode: type 0-100 to set target, 'auto' to exit");
            } else if (strncmp(buf, "bpr ", 4) == 0) {
                float bpr = atof(buf + 4);
                BoostController::setTargetBpr(bpr);
                Serial.print("BPR target set to ");
                Serial.println(BoostController::getTargetBpr());
            } else if (strcmp(buf, "pid") == 0) {
                float kp, ki, kd;
                BoostController::getTunings(kp, ki, kd);
                char tbuf[64];
                snprintf(tbuf, sizeof(tbuf), "PID tunings: Kp=%.2f Ki=%.2f Kd=%.2f",
                    (double)kp, (double)ki, (double)kd);
                Serial.println(tbuf);
            } else if (strncmp(buf, "pid ", 4) == 0) {
                float kp, ki, kd;
                if (sscanf(buf + 4, "%f %f %f", &kp, &ki, &kd) == 3) {
                    BoostController::setTunings(kp, ki, kd);
                    Serial.println("PID tunings updated");
                } else {
                    Serial.println("Usage: pid <Kp> <Ki> <Kd>");
                }
            } else {
                int val = atoi(buf);
                if (val >= 0 && val <= 100) {
                    if (BoostController::benchMode) {
                        BoostController::benchTarget = (float)val;
                        Serial.print("Bench target set to ");
                        Serial.print(val);
                        Serial.println("%");
                    } else {
                        manualPwm = (uint8_t)val;
                        manualMode = true;
                        appData.actuatorDemandedPosition = manualPwm;
                        Serial.print("Position set to ");
                        Serial.print(manualPwm);
                        Serial.println("%");
                    }
                } else {
                    Serial.println("Invalid: 0-100 or 'auto'");
                }
            }
            idx = 0;
        } else if (idx < 31) {
            buf[idx++] = c;
        }
    }
}

void ovgt::loop() {
    handleSerial();

    if (loopElapsed < 10) return;
    loopElapsed = 0;

    uint32_t t0, t1;
    count++;

    t0 = ARM_DWT_CYCCNT;
    handleDebug();
    t1 = ARM_DWT_CYCCNT;
    cyclesDebug += t1 - t0;

    appData.pgFault = digitalRead(PG_PIN);

    t0 = ARM_DWT_CYCCNT;
    AdcSensors::update();
    t1 = ARM_DWT_CYCCNT;
    cyclesAdc += t1 - t0;

    if (appData.boostPressureHpa > 500 && appData.boostPressureHpa < appData.ambientPressureGuessHpa) {
        appData.ambientPressureGuessHpa = appData.boostPressureHpa;
    }

    t0 = ARM_DWT_CYCCNT;
    if (!manualMode) {
        BoostController::update();
    }
    t1 = ARM_DWT_CYCCNT;
    cyclesBoost += t1 - t0;

    t0 = ARM_DWT_CYCCNT;
    Actuator::Loop();
    t1 = ARM_DWT_CYCCNT;
    cyclesActuator += t1 - t0;
}