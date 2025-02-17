#include "AppData.h"

class Actuator {
    public:
        static void Initialize(AppData *appData);
        static void Loop();
        static void SetPosition(uint8_t position);
        static AppData *appData;
};