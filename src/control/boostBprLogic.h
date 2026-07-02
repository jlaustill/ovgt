#ifndef boostBprLogic_h
#define boostBprLogic_h

#include <stdint.h>

struct BoostInputs {
    float boostGaugePsi;   // (COP - CIP) in psi
    float tipGaugePsi;     // turbine inlet (drive) pressure, gauge psi
};

struct BoostConfig {
    float   bprTarget;          // default 1.5
    float   boostSpoolPsi;      // fall back to spool BELOW this (hysteresis low)
    float   boostPiPsi;         // engage PI ABOVE this (hysteresis high)
    uint8_t spoolPercent;       // fixed vane position while spooling
    uint8_t vaneClosedPercent;  // from VaneConfig.h
    uint8_t vaneOpenPercent;    // from VaneConfig.h
    float   kp;
    float   ki;
    float   spoolProtectBoostPsi; // below this boost, PI may not open the vane past
                                  // the spool position (kills the handoff open-kick)
    uint8_t vaneOpenCapPercent;   // controller open-authority cap (< vaneOpenPercent):
                                  // the PI may never open the vane past this, blunting
                                  // the transient BPR proportional slam
};

struct BoostState {
    float integralTerm;  // accumulated integral CONTRIBUTION, in vane-% units
    bool  wasSpooling;   // pending spool->PI handoff seed (one-shot)
    bool  inPiRegion;    // hysteretic region: false = spool, true = PI
};

// Evaluate one boost control cycle and return the demanded vane position.
// Hysteresis around the spool/PI boundary (boostSpoolPsi..boostPiPsi) stops vane
// chatter when boost hovers at the threshold. In the spool region: hold
// cfg.spoolPercent (BPR is unreliable at low boost). In the PI region: drive
// BPR (= tipGaugePsi / boostGaugePsi) to cfg.bprTarget. Caller invokes this only
// when NOT in manual mode and NOT braking, so no mode/active flag is needed.
// Mutates state.
uint8_t boostBprStep(const BoostInputs &in, const BoostConfig &cfg,
                     BoostState &state, float dtSeconds);

#endif
