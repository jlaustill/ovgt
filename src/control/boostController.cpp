#include <Arduino.h>
#include "boostController.h"
#include "boostBprLogic.h"
#include "AppData.h"
#include "VaneConfig.h"

// ===== Boost control mode (compile-time hedge) =============================
// 1 = closed-loop BPR PI controller (drives drive/boost ratio to bprTarget).
// 0 = legacy open-loop boost->vane lookup map (the old tuning method).
// Flip to 0 and reflash to fall back to the map.
#define BOOST_USE_BPR 1

static const float HPA_TO_PSI = 0.0145038f;

// BPR controller config. ALL values are compile-time constants — to tune, edit
// here and reflash. There are deliberately NO runtime setters: changing tuning
// live over serial is a driving hazard. NOTE: BPR error is a 0..1 ratio, so kp is
// in the tens (vane-% per unit error), unlike the brake's psi-scale kp. kp/ki were
// validated on-truck (kp=88 was bang-bang; kp=20/ki=20 glides at sustained cruise).
// No boost ceiling: targets BPR only (deliberate — see design doc).
static BoostConfig boostConfig = {
    1.5f,                // bprTarget
    1.5f,                // boostSpoolPsi (fall back to spool below this)
    3.0f,                // boostPiPsi (engage PI above this; hysteresis dead band)
    22,                  // spoolPercent (fixed vane position while spooling)
    VANE_CLOSED_PERCENT, // vaneClosedPercent (max drive / boost)
    VANE_OPEN_PERCENT,   // vaneOpenPercent (relief / mechanical open limit)
    20.0f,               // kp
    20.0f,               // ki
    6.0f,                // spoolProtectBoostPsi (below this boost, don't open past spool)
    55                   // vaneOpenCapPercent (controller may not open past this;
                         // settled operation stays <=50%, so this clips only the kick)
};
static BoostState boostState = {0.0f, true, false};
static uint32_t lastUpdateMs = 0;

#if !BOOST_USE_BPR
// Legacy lookup table: gauge boost (psi) -> vane position (%). The old hand-tuned
// fallback, kept as a hedge. Edit + reflash to tune.
struct PressureMapEntry {
    float pressurePsi;
    uint8_t positionPercent;
};
static const PressureMapEntry pressureMap[] = {
    {0.0f, 25},
    {53.5f, 40}
};
static const uint8_t pressureMapSize = sizeof(pressureMap) / sizeof(pressureMap[0]);

static uint8_t interpolate(float pressurePsi) {
    if (pressurePsi <= pressureMap[0].pressurePsi) {
        return pressureMap[0].positionPercent;
    }
    if (pressurePsi >= pressureMap[pressureMapSize - 1].pressurePsi) {
        return pressureMap[pressureMapSize - 1].positionPercent;
    }
    for (uint8_t i = 1; i < pressureMapSize; i++) {
        if (pressurePsi <= pressureMap[i].pressurePsi) {
            float pLow = pressureMap[i - 1].pressurePsi;
            float pHigh = pressureMap[i].pressurePsi;
            uint8_t posLow = pressureMap[i - 1].positionPercent;
            uint8_t posHigh = pressureMap[i].positionPercent;
            return posLow + (uint8_t)((pressurePsi - pLow) / (pHigh - pLow) * (posHigh - posLow));
        }
    }
    return pressureMap[pressureMapSize - 1].positionPercent;
}
#endif

void BoostController::Initialize() {
    boostState.integralTerm = 0.0f;
    boostState.wasSpooling = true;
    boostState.inPiRegion = false;
    lastUpdateMs = millis();
#if BOOST_USE_BPR
    Serial.println("BoostController initialized (mode: BPR)");
#else
    Serial.println("BoostController initialized (mode: MAP)");
#endif
}

void BoostController::update() {
    uint32_t now = millis();
    float dt = (now - lastUpdateMs) / 1000.0f;
    lastUpdateMs = now;
    if (dt <= 0.0f) dt = 0.01f;

    int16_t boostGaugeHpa = (int16_t)appData.compressorOutputPressureHpaa
                          - (int16_t)appData.compressorInputPressureHpaa;
    if (boostGaugeHpa < 0) boostGaugeHpa = 0;
    float boostGaugePsi = boostGaugeHpa * HPA_TO_PSI;

#if BOOST_USE_BPR
    BoostInputs in;
    in.boostGaugePsi = boostGaugePsi;
    in.tipGaugePsi = appData.turbineInputPressureHpa * HPA_TO_PSI;
    appData.actuatorDemandedPosition = boostBprStep(in, boostConfig, boostState, dt);
#else
    (void)dt;
    appData.actuatorDemandedPosition = interpolate(boostGaugePsi);
#endif
}

float BoostController::getBprTarget() { return boostConfig.bprTarget; }
bool BoostController::getInPiRegion() { return boostState.inPiRegion; }
float BoostController::getIntegralTerm() { return boostState.integralTerm; }
