#include "AppData.h"

class ovgt
{
    private:
        static unsigned long count;
        static unsigned long lastMillis;
        static unsigned long thisMillis;
        static unsigned long thisDuration;
        static unsigned long loopCountLastMillis;
        static AppData appData;

    public:
        static void setup();
        static void loop();
};
