#ifndef AdcSensors_h
#define AdcSensors_h

#include "AppData.h"

class AdcSensors {
    public:
        static void Initialize();
        static void update();
    private:
        static uint8_t currentChannel;
        static bool conversionStarted;
        static uint32_t conversionStartTime;

        static void startConversion(uint8_t channel);
        static void processResult(uint8_t channel, float voltage);
        static float steinhartHart(float resistance);
};

#endif
