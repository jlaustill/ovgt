#ifndef exhaustBrakeLogic_h
#define exhaustBrakeLogic_h

#include <stdint.h>

struct BrakeInputs {
    uint8_t  torqueConverterLockupStatus;  // 0-3, 1 = engaged
    uint8_t  acceleratorPedalPercent;      // 0-100
    uint8_t  engineLoadPercent;            // 0-250
    float    tipGaugePsi;                  // measured backpressure (turbine inlet, gauge)
    uint32_t nowMs;
    uint32_t lastEec2RxMs;
    uint32_t lastEtc1RxMs;
    bool     manualMode;
};

struct BrakeConfig {
    float    targetPsi;
    float    ceilingPsi;
    uint8_t  vaneClosedPercent;  // most-closed position = max backpressure (0 on this actuator)
    uint8_t  vaneOpenPercent;    // fully-open position = relief / mechanical limit (68)
    uint32_t staleTimeoutMs;
    float    kp;
    float    ki;
};

struct BrakeState {
    float integralTerm;  // accumulated integral CONTRIBUTION, in vane-% units
};

struct BrakeOutput {
    bool    active;       // true when the brake owns the actuator this cycle
    uint8_t vanePercent;  // demanded vane position (valid when active)
    bool    fault;        // true when the backpressure ceiling was exceeded
};

// Evaluate one brake control cycle. Priority order: manual override, stale-CAN
// failsafe, instant throttle/load release, engage condition, ceiling cutoff,
// then the PI loop. Mutates state.integralTerm.
BrakeOutput exhaustBrakeStep(const BrakeInputs &in, const BrakeConfig &cfg,
                             BrakeState &state, float dtSeconds);

#endif
