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
    uint16_t compressorOutputPressureHpaa;
    float compressorInputTempC;
    uint16_t compressorInputPressureHpaa;
    uint16_t turbineInputPressureHpa;
    float turbineInputVoltage;
    float tipZeroVoltage;
    int16_t turbineInletTempC;
    float compressorOutputTempC;
    int16_t turbineOutletTempC;
    int16_t oilTempC;
    uint16_t oilPressureHpa;
    uint16_t turbineOutPressureHpa;
    uint16_t liftPumpPressureHpa;
    bool pgFault;
    // J1939-received engine/transmission state (decoded in j1939Sniff)
    uint8_t torqueConverterLockupStatus;  // SPN 573, 2-bit (0-3), 1 = engaged
    uint8_t acceleratorPedalPercent;      // SPN 91, 0-100
    uint8_t engineLoadPercent;            // SPN 92, 0-250
    volatile uint32_t lastEec2RxMs;       // millis() of last EEC2 (PGN 61443) receive
    volatile uint32_t lastEtc1RxMs;       // millis() of last ETC1 (PGN 61442) receive
    bool exhaustBrakeActive;              // true while the exhaust brake owns the actuator
};

extern AppData appData;

static const bool ACTUATOR_MODE_CAN = true;

#endif