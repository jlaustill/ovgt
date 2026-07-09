#include "cotThermistor.h"
#include <math.h>

static const float PULLUP_OHMS = 1000.0f;   // pull-up to 5 V, RIFE NTC to GND
static const float SUPPLY_VOLTS = 5.0f;
static const float RAIL_HIGH_VOLTS = 4.99f; // >= this reads as open circuit
static const float RAIL_LOW_VOLTS = 0.01f;  // <= this reads as short circuit

// RIFE Hi-AT Steinhart-Hart: 1/T(K) = A + B·lnR + C·lnR³ (0.11 °C fit over table).
static const float STEINHART_A = 6.535185e-4f;
static const float STEINHART_B = 2.345466e-4f;
static const float STEINHART_C = 9.380459e-11f;

bool cotThermistorReadC(float voltage, float *outTempC) {
    if (voltage >= RAIL_HIGH_VOLTS || voltage <= RAIL_LOW_VOLTS) {
        return false;
    }
    float resistance = PULLUP_OHMS * voltage / (SUPPLY_VOLTS - voltage);
    float lnR = logf(resistance);
    float invT = STEINHART_A + STEINHART_B * lnR + STEINHART_C * lnR * lnR * lnR;
    *outTempC = (1.0f / invT) - 273.15f;
    return true;
}
