#include <Arduino.h>
#include "ovgt.h"
#include <actuator.hpp>
#include <json.h>
#include "AppData.h"
#include "sensors/adcSensors.h"
#include "sensors/titSensor.h"
#include "sensors/cotSensor.h"
#include "sensors/citSensor.h"
#include "sensors/totSensor.h"
#include "sensors/compressorEfficiency.h"
#include "sensors/cotSettle.h"
#include "storage/fram.h"
#include "sensors/j1939.h"
#include "control/boostController.h"
#include "control/exhaustBrakeController.h"

elapsedMillis loopElapsed;
elapsedMicros adcElapsed;

AppData appData;

static CotSettleState cotSettleState;
static CotSettleConfig cotSettleCfg = {
    0.3f,   // cotSlopeFlatCperS
    0.5f,   // boostSlopeFlatPsiPerS
    2.0f,   // settledSeconds
    3.0f,   // minStepC
    512,    // maxBufferSamples
    0.5f    // slopeWindowSeconds (windowed flatness for the settled flag; rejects
            // K-type per-sample noise that otherwise pins the flag off — see
            // test_settled_flag_with_realistic_cot_noise)
};
static bool ceSettled = false;
static elapsedMicros cotSampleDt;

volatile uint32_t ovgt::count;

uint8_t ovgt::manualPwm = 0;
bool ovgt::manualMode = false;
uint32_t ovgt::cyclesAdc = 0;
uint32_t ovgt::cyclesBoost = 0;
uint32_t ovgt::cyclesActuator = 0;
uint32_t ovgt::cyclesDebug = 0;

void ovgt::handleDebug() {
    if (count % 10 != 0) return;  // emit at 10 Hz (loop runs at 100 Hz)

    int16_t boostGauge = (int16_t)appData.compressorOutputPressureHpaa
                       - (int16_t)appData.compressorInputPressureHpaa;
    if (boostGauge < 0) boostGauge = 0;
    float boostPsi = boostGauge * 0.0145038f;
    float pressureRatio = (appData.compressorInputPressureHpaa > 0)
        ? (float)appData.compressorOutputPressureHpaa / appData.compressorInputPressureHpaa
        : 0.0f;
    float bpr = (boostGauge != 0)
        ? (float)appData.turbineInputPressureHpa / boostGauge
        : 0.0f;
    float efficiency = compressorEfficiency(pressureRatio,
                                            (float)appData.compressorInputTempC,
                                            (float)appData.compressorOutputTempC);

    Json_begin();
    Json_addStr("type", "t");
    Json_addUint("t_ms", (uint32_t)millis());
    Json_addStr("mode", manualMode ? "manual" : (appData.exhaustBrakeActive ? "brake" : "auto"));
    Json_addUint("cop_hpa", appData.compressorOutputPressureHpaa);
    Json_addUint("cip_hpa", appData.compressorInputPressureHpaa);
    Json_addFloat2("boost_psi", boostPsi);
    Json_addFloat2("br", pressureRatio);
    Json_addFloat2("tip_psi", appData.turbineInputPressureHpa * 0.0145038f);
    Json_addFloat2("bpr", bpr);
    Json_addFloat2("bpr_target", BoostController::getBprTarget());
    // Controller-internal state, to diagnose low-load boost "stick": which region
    // the loop is in (spool vs PI) and the integral windup that precedes the snap.
    Json_addStr("boost_region", BoostController::getInPiRegion() ? "pi" : "spool");
    Json_addFloat2("boost_integ", BoostController::getIntegralTerm());
    Json_addFloat2("cit_c", appData.compressorInputTempC);
    Json_addFloat2("cot_c", appData.compressorOutputTempC);
    Json_addInt("tit_c", appData.turbineInletTempC);
    Json_addFloat2("ce_pct", efficiency >= 0.0f ? efficiency * 100.0f : -1.0f);
    Json_addBool("ce_settled", ceSettled);
    Json_addUint("dem_pct", manualMode ? manualPwm : appData.actuatorDemandedPosition);
    Json_addUint("pos_pct", appData.actuatorReportedPosition);
    Json_addBool("brake", appData.exhaustBrakeActive);
    Json_end();
    for (uint32_t i = 0; i < Json_len(); i++) Serial.write(Json_at(i));
    Serial.write('\n');
}


void ovgt::setup() {
    Serial.begin(115200);

    // Diagnostic: after a fault-induced reset the Teensy core retains a crash
    // report in no-init RAM. Printing it on boot tells us the fault type and
    // the program counter/address that crashed. If resets keep happening but
    // this stays blank, the cause is power/brownout, not a code fault.
    if (CrashReport) {
        Serial.println("=== CRASH REPORT ===");
        Serial.print(CrashReport);
        Serial.println("=== END CRASH REPORT ===");
    }

    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

    count = 0;
    pinMode(PG_PIN, INPUT);

    // Drive all SPI CS pins high before initializing any SPI device
    for (uint8_t pin : {3, 4, 5, 10, 24, 25, 26, 35, 37, 38, 39, 40}) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
    }

    AdcSensors::Initialize();
    TitSensor::Initialize();
    CotSensor::Initialize();
    // CitSensor::Initialize();
    // TotSensor::Initialize();
    Fram::Initialize();
    BoostController::Initialize();
    cotSettleInit(cotSettleState);
    ExhaustBrakeController::Initialize();
    Actuator_Initialize();
    J1939::Initialize();

    Serial.println("Setup complete");
    Serial.println("Type a number 0-100 to set vane position %, or 'auto' for normal operation");
    Serial.println("Tuning: 'bpr <v>', 'kp <v>', 'ki <v>', 'params' (BPR mode only)");
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
            } else if (strcmp(buf, "params") == 0) {
                BoostController::printParams();
            } else if (strncmp(buf, "bpr ", 4) == 0) {
                float value = atof(buf + 4);
                BoostController::setBprTarget(value);
                Serial.print("BPR target = ");
                Serial.println(value);
            } else if (strncmp(buf, "kp ", 3) == 0) {
                float value = atof(buf + 3);
                BoostController::setKp(value);
                Serial.print("kp = ");
                Serial.println(value);
            } else if (strncmp(buf, "ki ", 3) == 0) {
                float value = atof(buf + 3);
                BoostController::setKi(value);
                Serial.print("ki = ");
                Serial.println(value);
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

    AdcSensors::update();
    TitSensor::update();
    if (CotSensor::update()) {
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

    J1939::Loop();   // process RX first so brake/boost act on fresh CAN data

    // In manual mode neither controller may touch actuatorDemandedPosition,
    // otherwise the boost map clobbers the manually commanded vane position
    // every cycle. The serial handler owns the demand while manual is active.
    bool braking = ExhaustBrakeController::update(manualMode);
    if (!manualMode && !braking) {
        BoostController::update();
    }

    t0 = ARM_DWT_CYCCNT;
    Actuator_Loop();
    t1 = ARM_DWT_CYCCNT;
    cyclesActuator += t1 - t0;
}