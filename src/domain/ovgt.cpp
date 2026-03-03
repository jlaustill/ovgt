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

uint8_t ovgt::manualPwm = 0;
bool ovgt::manualMode = false;

void ovgt::handleDebugTimer() {
    lcdDisplay.updateDisplay(count);

    bool pgGood = digitalRead(PG_PIN);
    Serial.print("PG:");
    Serial.print(pgGood ? "OK" : "FAIL");
    Serial.print(" Boost:");
    Serial.print(appData.boostPressureHpa);
    Serial.print("hPa Dem:");
    Serial.print(appData.actuatorDemandedPosition);
    Serial.print("% Rep:");
    Serial.print(appData.actuatorReportedPosition);
    Serial.print("% Tmp:");
    Serial.print(appData.actuatorTemp);
    Serial.print("C S:");
    Serial.println(appData.actuatorStatus);
    count = 0;
}


void ovgt::setup() {
    Serial.begin(115200);
    lcdDisplay.init(&ovgt::appData);

    count = 0;
    pinMode(PG_PIN, INPUT);

    BoostSensor::Initialize(&ovgt::appData);
    BoostController::Initialize(&ovgt::appData);
    Actuator::Initialize(&ovgt::appData);

    debugTimer.begin(handleDebugTimer, 1 * 1000 * 1000); // 1s

    Serial.println("Setup complete");
    Serial.println("Type a number 0-247 to set PWM, or 'auto' for normal operation");
}

void ovgt::handleSerial() {
    static char buf[16];
    static uint8_t idx = 0;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (idx == 0) continue;
            buf[idx] = '\0';
            if (strcmp(buf, "auto") == 0) {
                manualMode = false;
                Serial.println("Switched to auto mode");
            } else {
                int val = atoi(buf);
                if (val >= 0 && val <= 247) {
                    manualPwm = (uint8_t)val;
                    manualMode = true;
                    Actuator::SetPWM(manualPwm);
                    Serial.print("PWM set to ");
                    Serial.println(manualPwm);
                } else {
                    Serial.println("Invalid: 0-247 or 'auto'");
                }
            }
            idx = 0;
        } else if (idx < 15) {
            buf[idx++] = c;
        }
    }
}

void ovgt::loop() {
    count++;
    handleSerial();

    if (manualMode) {
        // Manual PWM hold — actuator stays at last set value
        return;
    }

    // Safety: if 5V PSU power good is low, sensor data is unreliable
    if (!digitalRead(PG_PIN)) {
        appData.actuatorDemandedPosition = SAFE_VANE_POSITION;
        Actuator::Loop();
        return;
    }

    BoostSensor::read();
    BoostController::update();
    Actuator::Loop();
}