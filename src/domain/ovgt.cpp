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
    lcdDisplay.updateDisplay(count);

    Serial.print("L/S:");
    Serial.print(count);
    Serial.print(" Boost:");
    Serial.print(appData.boostPressureHpa);
    Serial.print("hPa Dem:");
    Serial.print(appData.actuatorDemandedPosition);
    Serial.print("% Rep:");
    Serial.print(appData.actuatorReportedPosition);
    Serial.print("% Tmp:");
    Serial.print(appData.actuatorTemp);
    Serial.print("C Load:");
    Serial.print(appData.actuatorMotorLoad);
    Serial.print(" S:");
    Serial.println(appData.actuatorStatus);
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

    // Calibration mode: sweep PWM 0-255 and log raw CAN feedback
    // Comment out and uncomment normal loop below when done
    Actuator::CalibrateLoop();

    // Normal operation:
    // BoostSensor::read();
    // BoostController::update();
    // Actuator::Loop();
}