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

// Spare analog-capable pin for the Vin divider. Pin 20 = A6, verified free vs
// the current pin map: SPI CS {3,4,5,10,24,25,26,35,37,38,39,40}, READY {21,41},
// PG 33, CAN2/J1939 {0,1}, CAN3/actuator {30,31}, I2C {18,19}.
// NOTE: pin 40 (previously used here) is the CIT MAX31856 chip-select (citSensor.cpp)
// and is driven OUTPUT-HIGH every boot — do NOT use it. CONFIRM 20 against the
// physical wiring before connecting the divider. Unused while OVGT_VIN_ENABLED = 0.
static const uint8_t OVGT_VIN_ADC_PIN = 20;

#endif
