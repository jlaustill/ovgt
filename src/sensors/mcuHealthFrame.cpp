#include "mcuHealthFrame.h"

// Narrow a u32 reading to u16 without ever hitting the 0xFFFF N/A sentinel.
static uint16_t capBelowSentinel(uint32_t value) {
    return value > 0xFFFEu ? 0xFFFEu : static_cast<uint16_t>(value);
}

McuHealth buildMcuHealth(float mcuTempC, uint8_t resetCauseCode,
                         uint32_t bootCount, uint32_t uptimeMinutes) {
    McuHealth h;
    h.temperatureC     = mcuTempC;
    h.temperatureValid = true;
    h.resetCause       = resetCauseCode;              // 0..8 matches library codes
    h.bootCount        = capBelowSentinel(bootCount);
    h.bootCountValid   = true;
    h.uptimeMinutes    = capBelowSentinel(uptimeMinutes);
    h.uptimeValid      = true;
    return h;
}
