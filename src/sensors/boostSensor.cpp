#include <Arduino.h>
#include "boostSensor.h"
#include <Adafruit_ADS1X15.h>

static Adafruit_ADS1115 ads;

AppData *BoostSensor::appData;
uint8_t BoostSensor::channel = 0;

void BoostSensor::Initialize(AppData *currentData, uint8_t ch) {
    appData = currentData;
    channel = ch;
    ads.setGain(GAIN_TWOTHIRDS); // +/-6.144V range for 5V sensor
    ads.begin();
    Serial.println("BoostSensor initialized");
}

void BoostSensor::read() {
    int16_t raw = ads.readADC_SingleEnded(channel);
    float voltage = ads.computeVolts(raw);

    // 10 bar MAP sensor: 0.5V = 0 hPa, 4.5V = 10000 hPa
    // hPa = (voltage - 0.5) * 2500
    float hPa = (voltage - 0.5f) * 2500.0f;

    // Clamp to valid range
    if (hPa < 0.0f) hPa = 0.0f;
    if (hPa > 10000.0f) hPa = 10000.0f;

    appData->boostPressureHpa = (uint16_t)hPa;
}
