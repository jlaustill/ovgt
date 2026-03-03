#include <Arduino.h>
#include "ovgt.h"
#include "display/actuator.h"
#include "AppData.h"
#include "display/lcdDisplay.h"
#include "sensors/boostSensor.h"
#include "control/boostController.h"

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
    Serial.print(" Boost: ");
    Serial.print(appData.boostPressureHpa);
    Serial.println(" hPa");
    count = 0;
}


void ovgt::setup() {
    Serial.begin(115200);
    lcdDisplay.init(&ovgt::appData);

    count = 0;
    
    BoostSensor::Initialize(&ovgt::appData);
    BoostController::Initialize(&ovgt::appData);
    Actuator::Initialize(&ovgt::appData);

    debugTimer.begin(handleDebugTimer, 1 * 1000 * 1000); // 1s

    Serial.println("Setup complete");
}

void ovgt::loop() {
    count++;

    BoostSensor::read();
    BoostController::update();
    Actuator::Loop();
}