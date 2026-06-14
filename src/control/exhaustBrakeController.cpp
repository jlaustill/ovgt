#include <Arduino.h>
#include "exhaustBrakeController.h"
#include "exhaustBrakeLogic.h"
#include "AppData.h"
#include "VaneConfig.h"

static const float HPA_TO_PSI = 0.0145038f;

// --- Tunable configuration -------------------------------------------------
// WARNING: a target of >= ~55 psi REQUIRES 60 psi heavy-duty exhaust valve
// springs. With stock springs the exhaust valves float at/near 60 psi, losing
// braking and risking valvetrain damage. ceilingPsi is the valve-float
// hardware-protection limit for the 2004 5.9L 24-valve Cummins (~65 psi).
// Vane positions come from VaneConfig.h (closed = max backpressure, open = relief).
// kp/ki are starting estimates — tune on the truck.
static BrakeConfig brakeConfig = {
    60.0f,               // targetPsi
    65.0f,               // ceilingPsi (valve-float limit)
    VANE_CLOSED_PERCENT, // vaneClosedPercent (max backpressure)
    VANE_OPEN_PERCENT,   // vaneOpenPercent (relief / mechanical open limit)
    250,                 // staleTimeoutMs
    2.0f,                // kp (%vane/psi) — must span the full 0..VANE_OPEN travel;
                         //   0.5 only reached ~1/3 closure so it jumped open then
                         //   crawled closed via the integral. Tune up (3-4) for a
                         //   harder bite, down if it overshoots/oscillates past target.
    0.3f                 // ki (%vane per psi/s)
};

static BrakeState brakeState = {0.0f};
static uint32_t lastUpdateMs = 0;

void ExhaustBrakeController::Initialize() {
    brakeState.integralTerm = 0.0f;
    lastUpdateMs = millis();
    Serial.println("ExhaustBrakeController initialized");
}

bool ExhaustBrakeController::update(bool manualMode) {
    uint32_t now = millis();
    float dt = (now - lastUpdateMs) / 1000.0f;
    lastUpdateMs = now;
    if (dt <= 0.0f) dt = 0.01f;

    BrakeInputs in;
    in.torqueConverterLockupStatus = appData.torqueConverterLockupStatus;
    in.acceleratorPedalPercent = appData.acceleratorPedalPercent;
    in.engineLoadPercent = appData.engineLoadPercent;
    in.tipGaugePsi = appData.turbineInputPressureHpa * HPA_TO_PSI;
    in.nowMs = now;
    in.lastEec2RxMs = appData.lastEec2RxMs;
    in.lastEtc1RxMs = appData.lastEtc1RxMs;
    in.manualMode = manualMode;

    BrakeOutput out = exhaustBrakeStep(in, brakeConfig, brakeState, dt);

    appData.exhaustBrakeActive = out.active;
    if (out.active) {
        appData.actuatorDemandedPosition = out.vanePercent;
    }
    return out.active;
}
