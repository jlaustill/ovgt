#ifndef SystemHealthConfig_h
#define SystemHealthConfig_h

#include <stdint.h>

// Compile-time only (driving hazard forbids runtime config).

// 1 = enable the IMXRT WDOG1 hardware watchdog; 0 = disabled (fast escape hatch
// if it ever false-trips into a boot loop). Dead code is eliminated when 0.
static const uint8_t OVGT_WATCHDOG_ENABLED = 1;

// Watchdog timeout via WDOG1_WCR.WT: timeout = (WT + 1) * 0.5 s. 3 -> 2.0 s.
// Conservative start; tighten from logged loop_us_max data.
static const uint8_t OVGT_WATCHDOG_WT = 3;

// 1 = a Vin resistor divider is fitted on OVGT_VIN_ADC_PIN; 0 = not fitted
// (vin_mv reports -1). Firmware ships with 0 until the divider is added.
static const uint8_t OVGT_VIN_ENABLED = 0;

// Spare analog-capable pin for the Vin divider (owner selects a free pin;
// pin 40 = A16 on Teensy 4.1). Unused while OVGT_VIN_ENABLED = 0.
static const uint8_t OVGT_VIN_ADC_PIN = 40;

#endif
