#ifndef BoostSensor_h
#define BoostSensor_h

#include "AppData.h"

class BoostSensor {
    public:
        static void Initialize(AppData *appData, uint8_t channel = 0);
        static void read();
    private:
        static AppData *appData;
        static uint8_t channel;
};

#endif
