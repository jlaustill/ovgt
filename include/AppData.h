#ifndef AppData_h
#define AppData_h

#include <Arduino.h>

struct AppData {
    uint8_t actuatorTemp;
    volatile uint8_t actuatorDemandedPosition;
    uint8_t actuatorReportedPosition;
    uint16_t actuatorRawPosition;
    uint16_t actuatorMotorLoad;
    uint8_t actuatorStatus;
    uint16_t boostPressureHpa;
    int16_t compressorInputTempC;
    uint16_t compressorInputPressureHpa;
    uint16_t turbineInputPressureHpa;
    float turbineInputVoltage;
    float tipZeroVoltage;
    int16_t turbineInletTempC;
    int16_t compressorOutputTempC;
    int16_t turbineOutletTempC;
    int16_t oilTempC;
    uint16_t oilPressureHpa;
    uint16_t turbineOutPressureHpa;
    uint16_t liftPumpPressureHpa;
    bool pgFault;
};

extern AppData appData;

static const bool ACTUATOR_MODE_CAN = true;

#endif