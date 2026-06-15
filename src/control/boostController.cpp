#include <Arduino.h>
#include "boostController.h"
#include "boostBprLogic.h"
#include "AppData.h"
#include "VaneConfig.h"

static const float HPA_TO_PSI = 0.0145038f;

// --- Tunable configuration -------------------------------------------------
// Closed-loop vane control targeting BPR (drive pressure / boost) = 1.0. Below
// boostMinPsi, BPR is unreliable (divide by ~0) so hold a fixed spool position.
// Vane limits come from VaneConfig.h. kp/ki/spoolPercent/boostMinPsi are starting
// estimates — tune on the truck. NOTE: BPR error is a 0..1 ratio, so kp is large
// (~full vane travel per unit error), unlike the brake's psi-scale kp.
// No boost ceiling: this targets BPR only (deliberate — see design doc).
static BoostConfig boostConfig = {
    1.0f,                // bprTarget
    2.0f,                // boostMinPsi (spool/BPR boundary)
    25,                  // spoolPercent (fixed vane position while spooling)
    VANE_CLOSED_PERCENT, // vaneClosedPercent (max drive / boost)
    VANE_OPEN_PERCENT,   // vaneOpenPercent (relief / mechanical open limit)
    88.0f,               // kp (%vane per unit BPR error)
    20.0f                // ki (%vane per (BPR error * s))
};

static BoostState boostState = {0.0f, true};
static uint32_t lastUpdateMs = 0;

void BoostController::Initialize() {
    boostState.integralTerm = 0.0f;
    boostState.wasSpooling = true;
    lastUpdateMs = millis();
    Serial.println("BoostController initialized");
}

void BoostController::update() {
    uint32_t now = millis();
    float dt = (now - lastUpdateMs) / 1000.0f;
    lastUpdateMs = now;
    if (dt <= 0.0f) dt = 0.01f;

    int16_t boostGaugeHpa = (int16_t)appData.compressorOutputPressureHpaa
                          - (int16_t)appData.compressorInputPressureHpaa;
    if (boostGaugeHpa < 0) boostGaugeHpa = 0;

    BoostInputs in;
    in.boostGaugePsi = boostGaugeHpa * HPA_TO_PSI;
    in.tipGaugePsi = appData.turbineInputPressureHpa * HPA_TO_PSI;

    appData.actuatorDemandedPosition = boostBprStep(in, boostConfig, boostState, dt);
}
