#ifndef AppData_h
#define AppData_h

#include <Arduino.h>

struct AppData {
    uint8_t actuatorTemp;
    uint8_t actuatorDemandedPosition;
    uint8_t actuatorReportedPosition;
    uint16_t actuatorRawPosition;
    uint16_t actuatorMotorLoad;
    uint8_t actuatorStatus;
    uint16_t boostPressureHpa;
    int16_t compressorInputTempC;
    uint16_t compressorInputPressureHpa;
    uint16_t turbineInputPressureHpa;
    float turbineInputVoltage;
    uint16_t ambientPressureGuessHpa;
};

extern AppData appData;

#endif