#ifndef BoostController_h
#define BoostController_h

#include "AppData.h"

struct PressureMapEntry {
    float pressurePsi;
    uint8_t positionPercent;
};

class BoostController {
    public:
        static void Initialize();
        static void update();
    private:
        static const PressureMapEntry pressureMap[];
        static const uint8_t pressureMapSize;
        static uint8_t interpolate(float pressurePsi);
};

#endif
