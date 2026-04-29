#ifndef Fram_h
#define Fram_h

#include <Arduino.h>

class Fram {
    public:
        static bool Initialize();
        static bool test();
};

#endif
