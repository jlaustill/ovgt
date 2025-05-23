#include "AppData.h"

class ovgt
{
    private:
        static uint32_t count;
        static AppData appData;
        static void handleDebugTimer();

    public:
        static void setup();
        static void loop();
};
