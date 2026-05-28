#include <Arduino.h>
#include "boostController.h"

// Tunable lookup table: gauge boost (PSI above ambient) -> vane position (0-100%)
// Edit these values and recompile to tune on the truck.
const PressureMapEntry BoostController::pressureMap[] = {
    {0.0f, 18},
    // {1.0f, 16},
    // {3.0f, 25},
    // {30.0f, 30},
    {53.5f, 30} // don't exceed 68% or the the actuator will try to go past the physical limit and cause damage
};
const uint8_t BoostController::pressureMapSize = sizeof(pressureMap) / sizeof(pressureMap[0]);

static const float HPA_TO_PSI = 0.0145038f;

void BoostController::Initialize() {
    Serial.println("BoostController initialized");
}

void BoostController::update() {
    int16_t gaugeBoostHpa = (int16_t)appData.compressorOutputPressureHpaa - (int16_t)appData.compressorInputPressureHpaa;
    if (gaugeBoostHpa < 0) gaugeBoostHpa = 0;
    float gaugeBoostPsi = gaugeBoostHpa * HPA_TO_PSI;
    uint8_t position = interpolate(gaugeBoostPsi);
    appData.actuatorDemandedPosition = position;
}

uint8_t BoostController::interpolate(float pressurePsi) {
    // Clamp below first entry
    if (pressurePsi <= pressureMap[0].pressurePsi) {
        return pressureMap[0].positionPercent;
    }
    // Clamp above last entry
    if (pressurePsi >= pressureMap[pressureMapSize - 1].pressurePsi) {
        return pressureMap[pressureMapSize - 1].positionPercent;
    }
    // Find bracketing entries and interpolate
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
