#include <Arduino.h>
#include <Adafruit_MAX31856.h>
#include "titSensor.h"

static const uint8_t CS_PIN   = 5;
static const uint8_t DRDY_PIN = 3;
static const uint8_t FLT_PIN  = 4;

static Adafruit_MAX31856 tc(CS_PIN);

void TitSensor::Initialize() {
    pinMode(DRDY_PIN, INPUT);
    pinMode(FLT_PIN, INPUT);
    tc.begin();
    tc.setThermocoupleType(MAX31856_TCTYPE_K);
    tc.setConversionMode(MAX31856_CONTINUOUS);
    Serial.println("TitSensor initialized");
}

void TitSensor::update() {
    if (digitalRead(DRDY_PIN)) return;  // wait for a fresh conversion
    uint8_t fault = tc.readFault();
    if (fault) {
        // Open circuit / out-of-range / etc.: reject the sample and hold the last
        // good value rather than store the railed reading. Rate-limit the print.
        static uint32_t lastFaultPrint = 0;
        if (millis() - lastFaultPrint > 1000) {
            lastFaultPrint = millis();
            Serial.print("TIT fault=0x");
            Serial.println(fault, HEX);
        }
        return;
    }
    appData.turbineInletTempC = (int16_t)tc.readThermocoupleTemperature();
}
