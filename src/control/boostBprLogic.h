#ifndef boostBprLogic_h
#define boostBprLogic_h

#include <stdint.h>

struct BoostInputs {
    float   boostGaugePsi;          // (COP - CIP) in psi
    float   tipGaugePsi;            // turbine inlet (drive) pressure, gauge psi
    uint8_t actuatorReportedPercent;// measured vane position (for anti-windup)
};

struct BoostConfig {
    float   bprTarget;              // 1.0
    float   boostSpoolPsi;          // fall back to spool BELOW this (hysteresis low)
    float   boostPiPsi;             // engage PI ABOVE this (hysteresis high)
    uint8_t spoolPercent;           // fixed vane position while spooling
    uint8_t vaneClosedPercent;      // from VaneConfig.h
    uint8_t vaneOpenPercent;        // from VaneConfig.h
    float   kp;
    float   ki;
    float   integralTrackingBand;   // freeze integral when |commanded - reported| > this (%vane)
};

struct BoostState {
    float   integralTerm;     // accumulated integral CONTRIBUTION, in vane-% units
    bool    wasSpooling;      // pending spool->PI handoff seed (one-shot)
    bool    inPiRegion;       // hysteretic region: false = spool, true = PI
    uint8_t lastVanePercent;  // last commanded vane (for actuator-tracking anti-windup)
};

// Evaluate one boost control cycle and return the demanded vane position.
// Hysteresis around the spool/PI boundary (boostSpoolPsi..boostPiPsi) stops vane
// chatter when boost hovers at the threshold. In the spool region: hold
// cfg.spoolPercent (BPR is unreliable at low boost). In the PI region: drive
// BPR (= tipGaugePsi / boostGaugePsi) to cfg.bprTarget. The integral only
// accumulates while the actuator has reached the last command (within
// integralTrackingBand), so it can't wind against a slewing/stuck actuator and
// dump as a tip-in spike. Caller invokes this only when NOT in manual mode and
// NOT braking, so no mode/active flag is needed. Mutates state.
uint8_t boostBprStep(const BoostInputs &in, const BoostConfig &cfg,
                     BoostState &state, float dtSeconds);

#endif
