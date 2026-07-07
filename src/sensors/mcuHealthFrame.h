#ifndef MCU_HEALTH_FRAME_H
#define MCU_HEALTH_FRAME_H

#include <stdint.h>
#include <J1939McuHealth.h>   // McuHealth struct + codec (jlaustill/J1939)

// Pure mapping: raw controller-health readings -> a fully-valid McuHealth.
// The two 16-bit fields are capped at 0xFFFE so a genuinely large boot count or
// uptime can never read as the 0xFFFF "not available" sentinel. resetCauseCode
// (0..8, already in the library's ordering) is passed through verbatim.
McuHealth buildMcuHealth(float mcuTempC, uint8_t resetCauseCode,
                         uint32_t bootCount, uint32_t uptimeMinutes);

#endif  // MCU_HEALTH_FRAME_H
