#include <Arduino.h>
#include <Adafruit_MAX31856.h>
#include "cotSensor.h"
#include "AppData.h"

static const uint8_t CS_PIN   = 37;
static const uint8_t DRDY_PIN = 35;
static const uint8_t FLT_PIN  = 36;

static Adafruit_MAX31856 tc(CS_PIN);

void CotSensor::Initialize() {
    pinMode(DRDY_PIN, INPUT);
    pinMode(FLT_PIN, INPUT);
    tc.begin();
    tc.setThermocoupleType(MAX31856_TCTYPE_K);
    tc.setConversionMode(MAX31856_CONTINUOUS);
    Serial.println("CotSensor initialized");
}

// Returns true only when a fresh, fault-free reading was stored (drives the
// ~10 Hz cotSettle feed in the main loop).
bool CotSensor::update() {
    if (digitalRead(DRDY_PIN)) return false;  // no fresh conversion
    uint8_t fault = tc.readFault();
    if (fault) {
        // Open circuit / out-of-range / etc.: reject the sample and hold the last
        // good value rather than store the railed reading (which would poison COT
        // and the compressor-efficiency calc). Rate-limit the diagnostic print.
        static uint32_t lastFaultPrint = 0;
        if (millis() - lastFaultPrint > 1000) {
            lastFaultPrint = millis();
            Serial.print("COT fault=0x");
            Serial.println(fault, HEX);
        }
        return false;
    }
    appData.compressorOutputTempC = tc.readThermocoupleTemperature();
    return true;
}
