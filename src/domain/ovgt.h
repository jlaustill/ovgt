#include "AppData.h"

class ovgt
{
    private:
        static const uint8_t PG_PIN = 33;
        static const uint8_t SAFE_VANE_POSITION = 95;
        static uint32_t count;
        static uint8_t manualPwm;
        static bool manualMode;
        static void handleDebugTimer();
        static void handleSerial();

    public:
        static void setup();
        static void loop();
};
