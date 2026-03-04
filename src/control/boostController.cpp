#include <Arduino.h>
#include "boostController.h"

AppData *BoostController::appData;

// Tunable lookup table: pressure (absolute hPa) -> vane position (0-100%)
// Edit these values and recompile to tune on the truck.
const PressureMapEntry BoostController::pressureMap[] = {
    {1000, 20},
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
    // Clamp below first entry
    if (pressureHpa <= pressureMap[0].pressureHpa) {
        return pressureMap[0].positionPercent;
    }
    // Clamp above last entry
    if (pressureHpa >= pressureMap[pressureMapSize - 1].pressureHpa) {
        return pressureMap[pressureMapSize - 1].positionPercent;
    }
    // Find bracketing entries and interpolate
    for (uint8_t i = 1; i < pressureMapSize; i++) {
        if (pressureHpa <= pressureMap[i].pressureHpa) {
            uint16_t pLow = pressureMap[i - 1].pressureHpa;
            uint16_t pHigh = pressureMap[i].pressureHpa;
            uint8_t posLow = pressureMap[i - 1].positionPercent;
            uint8_t posHigh = pressureMap[i].positionPercent;
            return posLow + (uint8_t)(((uint32_t)(pressureHpa - pLow) * (posHigh - posLow)) / (pHigh - pLow));
        }
    }
    return pressureMap[pressureMapSize - 1].positionPercent;
}
