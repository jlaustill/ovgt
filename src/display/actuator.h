#include "AppData.h"

class Actuator {
    public:
        static void Initialize(AppData *appData, uint8_t pin = 2);
        static void Loop();
        static void SetPosition(uint8_t position);
        static AppData *appData;
    private:
        static uint8_t actuatorPin;
};
