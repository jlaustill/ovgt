#ifndef AdcSensors_h
#define AdcSensors_h

#include "AppData.h"

class AdcSensors {
    public:
        static void Initialize();
        static void update();
        static bool cotSampleReady();
        static void clearCotSample();
    private:
        static const uint8_t NUM_CHANNELS = 4;
        static constexpr float EMA_ALPHA = 0.01f; // ~100-sample smoothing

        // ADS1 (0x48) — boost/CIP/CIT/TIP
        static uint8_t currentChannel;
        static bool conversionStarted;
        static uint32_t conversionStartTime;
        static float ema[NUM_CHANNELS];
        static bool emaInitialized[NUM_CHANNELS];

        // ADS2 (0x49) — TOP/oil pressure/oil temp/lift pump
        static uint8_t currentChannel2;
        static bool conversionStarted2;
        static uint32_t conversionStartTime2;
        static float ema2[NUM_CHANNELS];
        static bool emaInitialized2[NUM_CHANNELS];
        static bool cotFresh;   // set when ADS2 ch3 (COT) stores a fresh reading

        static void updateAds1();
        static void updateAds2();
        static void startConversion(uint8_t channel);
        static void processResult(uint8_t channel, float voltage);
        static void startConversion2(uint8_t channel);
        static void processResult2(uint8_t channel, float voltage);
        static float steinhartHart(float resistance);
        static float steinhartHartAem(float resistance);
};

#endif
