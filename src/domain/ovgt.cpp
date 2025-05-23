#include <Arduino.h>
#include "ovgt.h"
#include "display/actuator.h"
#include "AppData.h"
#include "display/lcdDisplay.h"

#include <SPI.h>

IntervalTimer debugTimer;

LcdDisplay lcdDisplay(0x27, 20, 4);

uint32_t ovgt::count;
AppData ovgt::appData;

void ovgt::handleDebugTimer() {
    // This function is called every 1s
    // You can add any debug code here
    lcdDisplay.updateDisplay(count);


    Serial.print("Loop count/Sec: ");
    Serial.print(count);
    Serial.println("");
    count = 0;
}


void ovgt::setup() {
    Serial.begin(115200);
    lcdDisplay.init(&ovgt::appData);

    count = 0;
    
    Actuator::Initialize(&ovgt::appData);

    debugTimer.begin(handleDebugTimer, 1 * 1000 * 1000); // 1s

    Serial.println("Setup complete");
}

void ovgt::loop() {
    count++;

    Actuator::Loop();
}