#include "Protocentral_ADS1220.h"
#include <SPI.h>

class Ads1220Wrapper {
    public:
        Ads1220Wrapper(uint8_t csPin, uint8_t dataReadyPin);

        static volatile bool dataReadyFlag;
        float Vout = 0.0;

        void setup();
        void loop();

    private:
        Protocentral_ADS1220 pc_ads1220;
        int32_t adc_data;
        float ADS1220Temperature;
        uint8_t csPin;
        uint8_t dataReadyPin;

        static void dataReadyHandler();
        void enableInterruptPin();
};