#include <Arduino.h>
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
    // TODO: User will implement interpolation logic
    // Clamp below first entry, clamp above last entry,
    // linearly interpolate between entries.
    return pressureMap[0].positionPercent;
}
