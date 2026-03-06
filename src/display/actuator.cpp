#include <Arduino.h>
#include "actuator.h"

#include <FlexCAN_T4.h>
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can0;

uint8_t Actuator::actuatorPin = 2; // Default to pin 2

// PWM calibration (output to actuator): 0% open = PWM 100, 100% open = PWM 247
static const uint8_t VANE_PWM_CLOSED = 100;
static const uint8_t VANE_PWM_OPEN = 247;

// CAN raw position feedback range (reported by actuator over CAN)
static const uint16_t VANE_CAN_RAW_CLOSED = 918;
static const uint16_t VANE_CAN_RAW_OPEN = 174;

void canSniff(const CAN_message_t &msg) {
    uint16_t rawPosition = (msg.buf[2] << 8) | msg.buf[3];
    uint16_t motorLoad = msg.buf[6] << 8 | msg.buf[7];
    uint8_t status = msg.buf[0];
    uint8_t temp = msg.buf[5];

    appData.actuatorRawPosition = rawPosition;
    appData.actuatorMotorLoad = motorLoad;
    appData.actuatorStatus = status;
    appData.actuatorTemp = temp;
    appData.actuatorReportedPosition = map(rawPosition, VANE_CAN_RAW_CLOSED, VANE_CAN_RAW_OPEN, 0, 100);
}

void Actuator::SetPosition(uint8_t position) {
    if (position > 100) {
        position = 100;
    }
    uint8_t pwm = map(position, 0, 100, VANE_PWM_CLOSED, VANE_PWM_OPEN);
    analogWrite(actuatorPin, pwm);
}

void Actuator::Initialize(uint8_t pin) {
    actuatorPin = pin;
    
    Can0.begin();
    Can0.setBaudRate(500 * 1000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    Can0.onReceive(canSniff);
    Can0.mailboxStatus();

    analogWriteFrequency(actuatorPin, 300.0);
    pinMode(actuatorPin, OUTPUT);
    SetPosition(appData.actuatorDemandedPosition);
}

void Actuator::SetPWM(uint8_t pwm) {
    analogWrite(actuatorPin, pwm);
}

void Actuator::Loop() {
    static uint32_t timeout = millis();
    if (millis() - timeout > 100) {
        SetPosition(appData.actuatorDemandedPosition);
        timeout = millis();
    }
}

void Actuator::CalibrateLoop() {
    static uint16_t pwmValue = 0;
    static uint32_t timeout = millis();
    static bool done = false;
    static bool headerPrinted = false;

    if (done) return;

    if (!headerPrinted) {
        Serial.println("PWM,RawPosition,Temp,MotorLoad");
        headerPrinted = true;
    }

    if (millis() - timeout > 1000) {
        // Log current PWM and raw feedback
        Serial.print(pwmValue);
        Serial.print(",");
        Serial.print(appData.actuatorRawPosition);
        Serial.print(",");
        Serial.print(appData.actuatorTemp);
        Serial.print(",");
        Serial.println(appData.actuatorMotorLoad);

        pwmValue++;
        if (pwmValue > 255) {
            Serial.println("Calibration complete");
            analogWrite(actuatorPin, 0);
            done = true;
            return;
        }

        analogWrite(actuatorPin, pwmValue);
        timeout = millis();
    }
}
