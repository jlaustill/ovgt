#include <Arduino.h>
#include "ovgt.h"
#include "display/actuator.h"

unsigned long ovgt::lastMillis = 0;
unsigned long ovgt::thisMillis;
unsigned long ovgt::thisDuration;
unsigned long ovgt::count;
unsigned long ovgt::loopCountLastMillis = 0;

void ovgt::setup() {
    Serial.begin(115200);

    lastMillis = millis();
    thisMillis = millis();
    count = 0;
    thisDuration = 0;
    
    Actuator::Initialize();
    Serial.println("Setup complete");
}

void ovgt::loop() {
    thisMillis = millis();
    thisDuration = thisMillis - lastMillis;
    count++;
    lastMillis = thisMillis;

    Actuator::Loop();

    if (thisMillis - loopCountLastMillis > 1000) {
        Serial.print("Loop count/Sec: ");
        Serial.println(count);
        count = 0;
        loopCountLastMillis = thisMillis;
    }
}