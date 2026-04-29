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
    static uint32_t last = 0;
    if (millis() - last > 1000) {
        last = millis();
        uint8_t f = tc.readFault();
        if (f) { Serial.print("TIT fault=0x"); Serial.println(f, HEX); }
    }
    if (digitalRead(DRDY_PIN)) return;
    appData.turbineInletTempC = (int16_t)tc.readThermocoupleTemperature();
}
