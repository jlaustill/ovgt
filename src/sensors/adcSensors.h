#ifndef AdcSensors_h
#define AdcSensors_h

#include "AppData.h"

class AdcSensors {
    public:
        static void Initialize();
        static void update();
    private:
        static const uint8_t NUM_CHANNELS = 4;
        static constexpr float EMA_ALPHA = 0.01f; // ~100-sample smoothing
        static uint8_t currentChannel;
        static bool conversionStarted;
        static uint32_t conversionStartTime;
        static float ema[NUM_CHANNELS];
        static bool emaInitialized[NUM_CHANNELS];

        static void startConversion(uint8_t channel);
        static void processResult(uint8_t channel, float voltage);
        static float steinhartHart(float resistance);
};

#endif
