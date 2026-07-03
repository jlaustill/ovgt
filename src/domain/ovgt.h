#include "AppData.h"

class ovgt
{
    private:
        static const uint8_t PG_PIN = 33;
        static volatile uint32_t count;
        static uint32_t cyclesAdc;
        static uint32_t cyclesBoost;
        static uint32_t cyclesActuator;
        static uint32_t cyclesDebug;
        static void handleDebug();
        static void handleJ1939Diag();
        static void handleJ1939Unknown();

    public:
        static void setup();
        static void loop();
};
