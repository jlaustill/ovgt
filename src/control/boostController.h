#ifndef BoostController_h
#define BoostController_h

#include "AppData.h"

struct PressureMapEntry {
    uint16_t pressureHpa;
    uint8_t positionPercent;
};

class BoostController {
    public:
        static void Initialize();
        static void update();
    private:
        static const PressureMapEntry pressureMap[];
        static const uint8_t pressureMapSize;
        static uint8_t interpolate(uint16_t pressureHpa);
};

#endif
