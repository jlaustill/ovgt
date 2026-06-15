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

// BPR controller config. bprTarget/kp/ki are runtime-tunable over serial (see
// setBprTarget/setKp/setKi) so the target (2.0 / 1.5 / 1.25 ...) and gains can be
// swept on the truck without reflashing. spoolPercent/boostMinPsi and the vane
// limits are compile-time. NOTE: BPR error is a 0..1 ratio, so kp is large
// (~full vane travel per unit error), unlike the brake's psi-scale kp.
// No boost ceiling: targets BPR only (deliberate — see design doc).
static BoostConfig boostConfig = {
    1.0f,                // bprTarget (runtime-tunable: `bpr <value>`)
    2.0f,                // boostMinPsi (spool/BPR boundary)
    25,                  // spoolPercent (fixed vane position while spooling)
    VANE_CLOSED_PERCENT, // vaneClosedPercent (max drive / boost)
    VANE_OPEN_PERCENT,   // vaneOpenPercent (relief / mechanical open limit)
    88.0f,               // kp (runtime-tunable: `kp <value>`)
    20.0f                // ki (runtime-tunable: `ki <value>`)
};
static BoostState boostState = {0.0f, true};
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

void BoostController::setBprTarget(float value) { boostConfig.bprTarget = value; }
void BoostController::setKp(float value) { boostConfig.kp = value; }
void BoostController::setKi(float value) { boostConfig.ki = value; }
float BoostController::getBprTarget() { return boostConfig.bprTarget; }

void BoostController::printParams() {
#if BOOST_USE_BPR
    Serial.print("boost mode: BPR  target=");
    Serial.print(boostConfig.bprTarget);
    Serial.print(" kp=");
    Serial.print(boostConfig.kp);
    Serial.print(" ki=");
    Serial.print(boostConfig.ki);
    Serial.print(" spool=");
    Serial.print(boostConfig.spoolPercent);
    Serial.print("% boostMin=");
    Serial.print(boostConfig.boostMinPsi);
    Serial.println("psi");
#else
    Serial.println("boost mode: MAP (legacy lookup table)");
#endif
}
