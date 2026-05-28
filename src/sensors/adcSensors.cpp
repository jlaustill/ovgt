#include <Arduino.h>
#include "adcSensors.h"
#include <Adafruit_ADS1X15.h>
#include <math.h>

static Adafruit_ADS1115 ads;
static Adafruit_ADS1115 ads2;

static const uint32_t CONVERSION_TIMEOUT_MS = 5;
static const uint8_t READY_PIN = 41;
static const uint8_t READY_PIN_2 = 21;
static const float TIP_ZERO_SAFETY_MIN = 0.25f;

// ADS1115 single-ended mux configs for channels 0-3
static const uint16_t MUX_CONFIG[] = {
    ADS1X15_REG_CONFIG_MUX_SINGLE_0,
    ADS1X15_REG_CONFIG_MUX_SINGLE_1,
    ADS1X15_REG_CONFIG_MUX_SINGLE_2,
    ADS1X15_REG_CONFIG_MUX_SINGLE_3
};

// GM 25037225 IAT sensor Steinhart-Hart coefficients
// Derived from published GM table: -20°C/28680Ω, 20°C/3520Ω, 80°C/310Ω
static const float SH_A = 0.0015728f;
static const float SH_B = 0.0002139f;
static const float SH_C = 0.0000001682f;

// AEM 30-2013 DTM fluid temp sensor Steinhart-Hart coefficients
// Derived from AEM calibration table (40/80/130°C points)
static const float AEM_SH_A = 0.001545f;
static const float AEM_SH_B = 0.0002138f;
static const float AEM_SH_C = 0.0000002313f;

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

static volatile bool conversionReady2 = false;
static void IRAM_ATTR ConversionReadyISR2() {
    conversionReady2 = true;
}

uint8_t AdcSensors::currentChannel = 0;
bool AdcSensors::conversionStarted = false;
uint32_t AdcSensors::conversionStartTime = 0;
float AdcSensors::ema[NUM_CHANNELS] = {0};
bool AdcSensors::emaInitialized[NUM_CHANNELS] = {false};

uint8_t AdcSensors::currentChannel2 = 0;
bool AdcSensors::conversionStarted2 = false;
uint32_t AdcSensors::conversionStartTime2 = 0;
float AdcSensors::ema2[NUM_CHANNELS] = {0};
bool AdcSensors::emaInitialized2[NUM_CHANNELS] = {false};

void AdcSensors::Initialize() {
    ads.setGain(GAIN_TWOTHIRDS); // +/-6.144V range for 5V sensors
    ads.setDataRate(RATE_ADS1115_860SPS);

    if (!ads.begin()) {
        Serial.println("ADS1115 #1 (0x48) not found, check wiring!");
        return;
    }

    pinMode(READY_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(READY_PIN), ConversionReadyISR, FALLING);

    currentChannel = 0;
    conversionStarted = false;
    appData.tipZeroVoltage = 0.5f;

    ads2.setGain(GAIN_TWOTHIRDS);
    ads2.setDataRate(RATE_ADS1115_860SPS);

    if (!ads2.begin(0x49)) {
        Serial.println("ADS1115 #2 (0x49) not found, check wiring!");
        return;
    }

    pinMode(READY_PIN_2, INPUT);
    attachInterrupt(digitalPinToInterrupt(READY_PIN_2), ConversionReadyISR2, FALLING);

    currentChannel2 = 0;
    conversionStarted2 = false;

    Serial.println("AdcSensors initialized (2x 4-channel round-robin)");
}

void AdcSensors::startConversion(uint8_t channel) {
    conversionReady = false;
    ads.startADCReading(MUX_CONFIG[channel], /*continuous=*/false);
    conversionStarted = true;
    conversionStartTime = millis();
}

void AdcSensors::startConversion2(uint8_t channel) {
    conversionReady2 = false;
    ads2.startADCReading(MUX_CONFIG[channel], /*continuous=*/false);
    conversionStarted2 = true;
    conversionStartTime2 = millis();
}

void AdcSensors::updateAds1() {
    if (!conversionStarted) {
        startConversion(currentChannel);
    } else if (millis() - conversionStartTime > CONVERSION_TIMEOUT_MS) {
        conversionStarted = false;
    } else if (conversionReady) {
        int16_t raw = ads.getLastConversionResults();
        float voltage = ads.computeVolts(raw);
        if (!emaInitialized[currentChannel]) {
            ema[currentChannel] = voltage;
            emaInitialized[currentChannel] = true;
        } else {
            ema[currentChannel] += EMA_ALPHA * (voltage - ema[currentChannel]);
        }
        processResult(currentChannel, ema[currentChannel]);
        conversionStarted = false;
        currentChannel = (currentChannel + 1) % NUM_CHANNELS;
    }
}

void AdcSensors::updateAds2() {
    if (!conversionStarted2) {
        startConversion2(currentChannel2);
    } else if (millis() - conversionStartTime2 > CONVERSION_TIMEOUT_MS) {
        conversionStarted2 = false;
    } else if (conversionReady2) {
        int16_t raw2 = ads2.getLastConversionResults();
        float voltage2 = ads2.computeVolts(raw2);
        if (!emaInitialized2[currentChannel2]) {
            ema2[currentChannel2] = voltage2;
            emaInitialized2[currentChannel2] = true;
        } else {
            ema2[currentChannel2] += EMA_ALPHA * (voltage2 - ema2[currentChannel2]);
        }
        processResult2(currentChannel2, ema2[currentChannel2]);
        conversionStarted2 = false;
        currentChannel2 = (currentChannel2 + 1) % NUM_CHANNELS;
    }
}

void AdcSensors::update() {
    updateAds1();
    updateAds2();
}

void AdcSensors::processResult(uint8_t channel, float voltage) {
    switch (channel) {
        case 0: {
            // Boost pressure: 10 bar MAP, 0.5–4.5V
            // hPa = (V - 0.5) * 2500
            float hPa = (voltage - 0.5f) * 2500.0f;
            if (hPa < 0.0f) hPa = 0.0f;
            if (hPa > 10000.0f) hPa = 10000.0f;
            appData.compressorOutputPressureHpaa = (uint16_t)hPa;
            break;
        }
        case 1: {
            // Compressor input pressure: 1 bar (15 PSI), 0.5–4.5V
            // hPa = (V - 0.5) * 258.575
            float hPa = (voltage - 0.5f) * 258.575f;
            if (hPa < 0.0f) hPa = 0.0f;
            if (hPa > 1034.0f) hPa = 1034.0f;
            appData.compressorInputPressureHpaa = (uint16_t)hPa;
            break;
        }
        case 2: {
            // Compressor input temp: GM NTC (25037225 family) — 2.2kΩ pullup from 5V, NTC to GND
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
            // Turbine input pressure: 100 PSIG, 0.5–4.5V (nominal)
            // Zero offset tracked per-session to handle sensor tolerance
            appData.turbineInputVoltage = voltage;
            if (voltage < appData.tipZeroVoltage && voltage >= TIP_ZERO_SAFETY_MIN) {
                appData.tipZeroVoltage = voltage;
            }
            float hPa = (voltage - appData.tipZeroVoltage) * 1723.69f;
            if (hPa < 0.0f) hPa = 0.0f;
            if (hPa > 6895.0f) hPa = 6895.0f;
            appData.turbineInputPressureHpa = (uint16_t)hPa;
            break;
        }
    }
}

void AdcSensors::processResult2(uint8_t channel, float voltage) {
    switch (channel) {
        case 0: {
            // TOP — turbine out pressure, 0.5–4.5V ratiometric
            // TODO: verify sensor PSI rating; using 30 PSI (2068 hPa) as default
            float hPa = (voltage - 0.5f) * 517.0f;
            if (hPa < 0.0f) hPa = 0.0f;
            if (hPa > 2068.0f) hPa = 2068.0f;
            appData.turbineOutPressureHpa = (uint16_t)hPa;
            break;
        }
        case 1: {
            // Oil pressure: 150 PSI (10342 hPa), 0.5–4.5V ratiometric
            float hPa = (voltage - 0.5f) * 2585.535f;
            if (hPa < 0.0f) hPa = 0.0f;
            if (hPa > 10342.0f) hPa = 10342.0f;
            appData.oilPressureHpa = (uint16_t)hPa;
            break;
        }
        case 2: {
            // Oil temp: AEM 30-2013 DTM sensor, 2.2kΩ pullup to 5V
            if (voltage >= NTC_VCC - 0.01f || voltage <= 0.01f) {
                break;
            }
            float resistance = PULLDOWN_R * voltage / (NTC_VCC - voltage);
            appData.oilTempC = (int16_t)steinhartHartAem(resistance);
            break;
        }
        case 3: {
            // Fuel lift pump pressure: 15 PSI (1034 hPa), 0.5–4.5V ratiometric
            float hPa = (voltage - 0.5f) * 258.575f;
            if (hPa < 0.0f) hPa = 0.0f;
            if (hPa > 1034.0f) hPa = 1034.0f;
            appData.liftPumpPressureHpa = (uint16_t)hPa;
            break;
        }
    }
}

float AdcSensors::steinhartHart(float resistance) {
    float lnR = logf(resistance);
    float invT = SH_A + SH_B * lnR + SH_C * lnR * lnR * lnR;
    return (1.0f / invT) - 273.15f;
}

float AdcSensors::steinhartHartAem(float resistance) {
    float lnR = logf(resistance);
    float invT = AEM_SH_A + AEM_SH_B * lnR + AEM_SH_C * lnR * lnR * lnR;
    return (1.0f / invT) - 273.15f;
}
