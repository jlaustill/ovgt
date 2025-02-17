#ifndef AppData_h
#define AppData_h

#include <Arduino.h>

struct AppData {
    uint8_t actuatorTemp;
    uint8_t actuatorDemandedPosition;
    uint8_t actuatorReportedPosition;
    uint16_t actuatorMotorLoad;
    uint8_t actuatorStatus;
};

#endif