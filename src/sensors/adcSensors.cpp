#include <Arduino.h>
#include "adcSensors.h"
#include <Adafruit_ADS1X15.h>
#include <math.h>

static Adafruit_ADS1115 ads;

static const uint8_t NUM_CHANNELS = 4;
static const uint32_t CONVERSION_TIMEOUT_MS = 15;
static const uint8_t READY_PIN = 41;

// ADS1115 single-ended mux configs for channels 0-3
static const uint16_t MUX_CONFIG[] = {
    ADS1X15_REG_CONFIG_MUX_SINGLE_0,
    ADS1X15_REG_CONFIG_MUX_SINGLE_1,
    ADS1X15_REG_CONFIG_MUX_SINGLE_2,
    ADS1X15_REG_CONFIG_MUX_SINGLE_3
};

// GM IAT Steinhart-Hart coefficients (25037225 family)
static const float SH_A = 0.001468f;
static const float SH_B = 0.000239f;
static const float SH_C = 0.0000001013f;

// Pulldown resistor value for NTC divider (ohms)
static const float PULLDOWN_R = 2200.0f;
// Supply voltage for NTC divider
static const float NTC_VCC = 5.0f;

// ISR flag — set by DRDY falling edge when conversion completes
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

static volatile bool conversionReady = false;
static void IRAM_ATTR ConversionReadyISR() {
    conversionReady = true;
}

uint8_t AdcSensors::currentChannel = 0;
bool AdcSensors::conversionStarted = false;
uint32_t AdcSensors::conversionStartTime = 0;

void AdcSensors::Initialize() {
    ads.setGain(GAIN_TWOTHIRDS); // +/-6.144V range for 5V sensors
    ads.begin();

    pinMode(READY_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(READY_PIN), ConversionReadyISR, FALLING);

    currentChannel = 0;
    conversionStarted = false;

    Serial.println("AdcSensors initialized (4-channel round-robin)");
}

void AdcSensors::startConversion(uint8_t channel) {
    conversionReady = false;
    ads.startADCReading(MUX_CONFIG[channel], /*continuous=*/false);
    conversionStarted = true;
    conversionStartTime = millis();
}

void AdcSensors::update() {
    if (!conversionStarted) {
        startConversion(currentChannel);
        return;
    }

    // Check for timeout
    if (millis() - conversionStartTime > CONVERSION_TIMEOUT_MS) {
        conversionStarted = false;
        currentChannel = (currentChannel + 1) % NUM_CHANNELS;
        return;
    }

    // Wait for ISR to signal conversion complete
    if (!conversionReady) {
        return;
    }

    // Read result and process
    int16_t raw = ads.getLastConversionResults();
    float voltage = ads.computeVolts(raw);
    processResult(currentChannel, voltage);

    // Advance to next channel
    conversionStarted = false;
    currentChannel = (currentChannel + 1) % NUM_CHANNELS;
}

void AdcSensors::processResult(uint8_t channel, float voltage) {
    switch (channel) {
        case 0: {
            // Boost pressure: 10 bar MAP, 0.5–4.5V
            // hPa = (V - 0.5) * 2500
            float hPa = (voltage - 0.5f) * 2500.0f;
            if (hPa < 0.0f) hPa = 0.0f;
            if (hPa > 10000.0f) hPa = 10000.0f;
            appData.boostPressureHpa = (uint16_t)hPa;
            break;
        }
        case 1: {
            // Compressor input pressure: 1 bar (15 PSI), 0.5–4.5V
            // hPa = (V - 0.5) * 258.575
            float hPa = (voltage - 0.5f) * 258.575f;
            if (hPa < 0.0f) hPa = 0.0f;
            if (hPa > 1034.0f) hPa = 1034.0f;
            appData.compressorInputPressureHpa = (uint16_t)hPa;
            break;
        }
        case 2: {
            // Compressor input temp: GM NTC + 2.2kΩ pulldown to GND, 5V supply
            // R = 2200 * V / (5.0 - V)
            if (voltage >= NTC_VCC - 0.01f || voltage <= 0.01f) {
                // Open or shorted sensor — don't update
                break;
            }
            float resistance = PULLDOWN_R * voltage / (NTC_VCC - voltage);
            appData.compressorInputTempC = (int16_t)steinhartHart(resistance);
            break;
        }
        case 3: {
            // Turbine input pressure: 150 PSI, 0.5–4.5V
            // hPa = (V - 0.5) * 2585.75
            float hPa = (voltage - 0.5f) * 2585.75f;
            if (hPa < 0.0f) hPa = 0.0f;
            if (hPa > 10343.0f) hPa = 10343.0f;
            appData.turbineInputPressureHpa = (uint16_t)hPa;
            break;
        }
    }
}

float AdcSensors::steinhartHart(float resistance) {
    float lnR = logf(resistance);
    float invT = SH_A + SH_B * lnR + SH_C * lnR * lnR * lnR;
    return (1.0f / invT) - 273.15f;
}
