#ifndef VaneConfig_h
#define VaneConfig_h

#include <stdint.h>

// VGT vane actuator travel calibration (actuator position %, 0-100 scale).
// SINGLE SOURCE OF TRUTH — reference these constants instead of hardcoding the
// limit anywhere (boost map, exhaust brake, actuator clamp).
//   VANE_CLOSED_PERCENT = fully CLOSED -> maximum backpressure / boost
//   VANE_OPEN_PERCENT   = fully OPEN   -> no restriction (mechanical stop / relief)
//
// Recalibrate if the actuator's range of motion changes. 2026-06-14: cleaning the
// vane pack freed up travel, raising the open stop from ~68 to 88.
static const uint8_t VANE_CLOSED_PERCENT = 0;
static const uint8_t VANE_OPEN_PERCENT = 88;

#endif
