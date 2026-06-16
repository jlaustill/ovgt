#include "compressorEfficiency.h"
#include <math.h>

static const float KELVIN_OFFSET = 273.15f;
static const float GAMMA_EXPONENT = 0.286f;     // (gamma-1)/gamma for air, gamma=1.4
static const float MIN_PRESSURE_RATIO = 1.0f;   // need real compression: BR (Po/Pi) > 1.0
static const float MIN_RISE_KELVIN = 1.0f;      // guards the 0/0 / sensor-noise region

float compressorEfficiency(float prRatio, float citC, float cotC) {
    // Below/at unity pressure ratio the numerator is <= 0 (no compression) -> undefined.
    if (prRatio <= MIN_PRESSURE_RATIO) {
        return -1.0f;
    }
    float inletKelvin = citC + KELVIN_OFFSET;
    float outletKelvin = cotC + KELVIN_OFFSET;
    float actualRise = outletKelvin - inletKelvin;
    // Guards the 0/0 indeterminate at idle and sensor noise near zero rise.
    if (actualRise < MIN_RISE_KELVIN) {
        return -1.0f;
    }
    float idealRise = inletKelvin * (powf(prRatio, GAMMA_EXPONENT) - 1.0f);
    return idealRise / actualRise;
}
