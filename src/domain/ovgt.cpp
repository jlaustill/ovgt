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
#include "sensors/j1939Health.h"
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

uint32_t ovgt::cyclesAdc = 0;
uint32_t ovgt::cyclesBoost = 0;
uint32_t ovgt::cyclesActuator = 0;
uint32_t ovgt::cyclesDebug = 0;

// Per-signal 5-state status from AppData freshness + raw J1939 status.
static J1939Status jStatus(bool domainOnline, uint32_t lastSeen, uint32_t timeoutMs, uint8_t rawStatus) {
    return j1939SignalStatus(domainOnline, lastSeen, millis(), timeoutMs, (J1939RawStatus)rawStatus);
}
static bool okJ(bool domainOnline, uint32_t lastSeen, uint32_t timeoutMs, uint8_t rawStatus) {
    return jStatus(domainOnline, lastSeen, timeoutMs, rawStatus) == J1939_STATUS_OK;
}
// Emit a decoded J1939 value, or null when its status is not "ok" this sample.
static void addUintOrNull(const char *key, bool ok, uint32_t v) {
    if (ok) Json_addUint(key, v); else Json_addNull(key);
}
static void addIntOrNull(const char *key, bool ok, int32_t v) {
    if (ok) Json_addInt(key, v); else Json_addNull(key);
}
static void addFloat2OrNull(const char *key, bool ok, float v) {
    if (ok) Json_addFloat2(key, v); else Json_addNull(key);
}

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
    Json_addStr("mode", appData.exhaustBrakeActive ? "brake" : "auto");
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
    Json_addFloat2("mcu_c", tempmonGetTemp());  // Teensy 4.1 on-die temperature
    Json_addFloat2("ce_pct", efficiency >= 0.0f ? efficiency * 100.0f : -1.0f);
    Json_addBool("ce_settled", ceSettled);
    Json_addUint("dem_pct", appData.actuatorDemandedPosition);
    Json_addUint("pos_pct", appData.actuatorReportedPosition);
    Json_addBool("brake", appData.exhaustBrakeActive);

    // Decoded J1939 broadcast signals (value when status ok, else null).
    uint32_t nowMs = millis();
    bool eng = j1939DomainOnline(appData.lastEec1RxMs, appData.lastEec2RxMs, nowMs, 1000);
    bool trn = j1939DomainOnline(appData.lastEtc1RxMs, appData.lastEtc2RxMs, nowMs, 1000);
    addUintOrNull("engine_rpm",        okJ(eng, appData.lastEec1RxMs, 1000, appData.engineRpmRaw),   appData.engineRpm);
    addIntOrNull("torque_pct",         okJ(eng, appData.lastEec1RxMs, 1000, appData.torqueRaw),      appData.actualTorquePct);
    addIntOrNull("torque_demand_pct",  okJ(eng, appData.lastEec1RxMs, 1000, appData.torqueRaw),      appData.driverDemandTorquePct);
    addUintOrNull("accel_pct",         okJ(eng, appData.lastEec2RxMs, 1000, J1939_OK),               appData.acceleratorPedalPercent);
    addUintOrNull("load_pct",          okJ(eng, appData.lastEec2RxMs, 1000, J1939_OK),               appData.engineLoadPercent);
    addIntOrNull("intake_air_c",       okJ(eng, appData.lastIc1RxMs, 1500, appData.intakeAirRaw),    appData.intakeAirTempC);
    addUintOrNull("j1939_boost_kpa",   okJ(eng, appData.lastIc1RxMs, 1500, appData.boostRaw),        appData.j1939BoostKpa);
    addUintOrNull("preturbo_kpa",      okJ(eng, appData.lastAmbRxMs, 3000, appData.preTurboRaw),     appData.preTurboKpa);
    addFloat2OrNull("system_v",        okJ(eng, appData.lastVep1RxMs, 3000, appData.systemVoltageRaw), appData.systemVoltage);
    addIntOrNull("coolant_c",          okJ(eng, appData.lastEt1RxMs, 3000, appData.coolantRaw),      appData.coolantTempC);
    addIntOrNull("oil_c",              okJ(eng, appData.lastEt1RxMs, 3000, appData.oilTempRaw),      appData.engineOilTempC);
    addUintOrNull("oil_kpa",           okJ(eng, appData.lastEflRxMs, 1500, appData.oilPressRaw),     appData.oilPressureKpaJ1939);
    addUintOrNull("tcc",               okJ(trn, appData.lastEtc1RxMs, 1000, J1939_OK),               appData.torqueConverterLockupStatus);
    addUintOrNull("trans_out_rpm",     okJ(trn, appData.lastEtc1RxMs, 1000, appData.outputShaftRaw), appData.outputShaftRpm);
    addUintOrNull("trans_in_rpm",      okJ(trn, appData.lastEtc1RxMs, 1000, appData.inputShaftRaw),  appData.inputShaftRpm);
    addUintOrNull("clutch_slip_pct",   okJ(trn, appData.lastEtc1RxMs, 1000, appData.clutchSlipRaw),  appData.clutchSlipPct);
    addUintOrNull("gear_sel",          okJ(trn, appData.lastEtc2RxMs, 1000, appData.selectedGearRaw), appData.selectedGear);
    addUintOrNull("gear_cur",          okJ(trn, appData.lastEtc2RxMs, 1000, appData.currentGearRaw), appData.currentGear);
    addUintOrNull("gear_ratio",        okJ(trn, appData.lastEtc2RxMs, 1000, appData.gearRatioRaw),   appData.gearRatioMilli);
    Json_end();
    for (uint32_t i = 0; i < Json_len(); i++) Serial.write(Json_at(i));
    Serial.write('\n');
}

void ovgt::handleJ1939Diag() {
    if (count % 100 != 0) return;  // 1 Hz (loop runs at 100 Hz)
    uint32_t now = millis();
    bool eng = j1939DomainOnline(appData.lastEec1RxMs, appData.lastEec2RxMs, now, 1000);
    bool trn = j1939DomainOnline(appData.lastEtc1RxMs, appData.lastEtc2RxMs, now, 1000);

    Json_begin();
    Json_addStr("type", "d");
    Json_addUint("t_ms", now);
    Json_addBool("engine_online", eng);
    Json_addBool("trans_online", trn);
    Json_addUint("engine_up_ms", appData.engineOnlineAtMs);
    Json_addUint("trans_up_ms", appData.transOnlineAtMs);
    Json_addStr("h_engine_rpm",  j1939StatusName(jStatus(eng, appData.lastEec1RxMs, 1000, appData.engineRpmRaw)));
    Json_addStr("h_torque",      j1939StatusName(jStatus(eng, appData.lastEec1RxMs, 1000, appData.torqueRaw)));
    Json_addStr("h_intake_air",  j1939StatusName(jStatus(eng, appData.lastIc1RxMs, 1500, appData.intakeAirRaw)));
    Json_addStr("h_boost",       j1939StatusName(jStatus(eng, appData.lastIc1RxMs, 1500, appData.boostRaw)));
    Json_addStr("h_preturbo",    j1939StatusName(jStatus(eng, appData.lastAmbRxMs, 3000, appData.preTurboRaw)));
    Json_addStr("h_system_v",    j1939StatusName(jStatus(eng, appData.lastVep1RxMs, 3000, appData.systemVoltageRaw)));
    Json_addStr("h_coolant",     j1939StatusName(jStatus(eng, appData.lastEt1RxMs, 3000, appData.coolantRaw)));
    Json_addStr("h_oil_c",       j1939StatusName(jStatus(eng, appData.lastEt1RxMs, 3000, appData.oilTempRaw)));
    Json_addStr("h_oil_kpa",     j1939StatusName(jStatus(eng, appData.lastEflRxMs, 1500, appData.oilPressRaw)));
    Json_addStr("h_trans_out",   j1939StatusName(jStatus(trn, appData.lastEtc1RxMs, 1000, appData.outputShaftRaw)));
    Json_addStr("h_trans_in",    j1939StatusName(jStatus(trn, appData.lastEtc1RxMs, 1000, appData.inputShaftRaw)));
    Json_addStr("h_clutch_slip", j1939StatusName(jStatus(trn, appData.lastEtc1RxMs, 1000, appData.clutchSlipRaw)));
    Json_addStr("h_gear_sel",    j1939StatusName(jStatus(trn, appData.lastEtc2RxMs, 1000, appData.selectedGearRaw)));
    Json_addStr("h_gear_cur",    j1939StatusName(jStatus(trn, appData.lastEtc2RxMs, 1000, appData.currentGearRaw)));
    Json_addStr("h_gear_ratio",  j1939StatusName(jStatus(trn, appData.lastEtc2RxMs, 1000, appData.gearRatioRaw)));
    Json_addUint("unk_n", J1939::unknownCount());
    Json_addUint("unk_dropped", J1939::unknownDroppedCount());
    Json_end();
    for (uint32_t i = 0; i < Json_len(); i++) Serial.write(Json_at(i));
    Serial.write('\n');
}

void ovgt::handleJ1939Unknown() {
    if (count % 100 != 0) return;  // 1 Hz, aligned with the diagnostic line
    static uint8_t cursor = 0;
    uint8_t n = J1939::unknownCount();
    if (n == 0) return;
    uint8_t emit = n < 4 ? n : 4;  // up to 4/s -> full 32-entry table in 8 s
    for (uint8_t k = 0; k < emit; k++) {
        uint8_t idx = (uint8_t)((cursor + k) % n);
        uint32_t pgn, cnt, firstMs, lastMs; uint8_t sa, b[8];
        if (!J1939::unknownAt(idx, pgn, sa, cnt, firstMs, lastMs, b)) continue;
        static const char HEXD[] = "0123456789ABCDEF";
        char hex[17];
        for (uint8_t j = 0; j < 8; j++) { hex[j*2] = HEXD[b[j] >> 4]; hex[j*2+1] = HEXD[b[j] & 0xF]; }
        hex[16] = '\0';
        uint32_t spanMs = lastMs - firstMs;
        uint32_t hz = spanMs > 0 ? (cnt * 1000) / spanMs : 0;
        Json_begin();
        Json_addStr("type", "u");
        Json_addUint("t_ms", (uint32_t)millis());
        Json_addUint("pgn", pgn);
        Json_addUint("sa", sa);
        Json_addUint("cnt", cnt);
        Json_addUint("hz", hz);
        Json_addStr("last", hex);
        Json_end();
        for (uint32_t i = 0; i < Json_len(); i++) Serial.write(Json_at(i));
        Serial.write('\n');
    }
    cursor = (uint8_t)((cursor + emit) % n);
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
}

void ovgt::loop() {
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
    handleJ1939Diag();
    handleJ1939Unknown();
    t1 = ARM_DWT_CYCCNT;
    cyclesDebug += t1 - t0;

    appData.pgFault = digitalRead(PG_PIN);

    J1939::Loop();   // process RX first so brake/boost act on fresh CAN data

    bool braking = ExhaustBrakeController::update(false);
    if (!braking) {
        BoostController::update();
    }

    t0 = ARM_DWT_CYCCNT;
    Actuator_Loop();
    t1 = ARM_DWT_CYCCNT;
    cyclesActuator += t1 - t0;
}