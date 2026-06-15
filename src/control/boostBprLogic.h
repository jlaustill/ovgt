#ifndef boostBprLogic_h
#define boostBprLogic_h

#include <stdint.h>

struct BoostInputs {
    float boostGaugePsi;   // (COP - CIP) in psi
    float tipGaugePsi;     // turbine inlet (drive) pressure, gauge psi
};

struct BoostConfig {
    float   bprTarget;          // 1.0
    float   boostMinPsi;        // below this -> spool region (BPR unreliable)
    uint8_t spoolPercent;       // fixed vane position while spooling
    uint8_t vaneClosedPercent;  // from VaneConfig.h
    uint8_t vaneOpenPercent;    // from VaneConfig.h
    float   kp;
    float   ki;
};

struct BoostState {
    float integralTerm;  // accumulated integral CONTRIBUTION, in vane-% units
    bool  wasSpooling;   // true while in the spool region; drives bumpless handoff
};

// Evaluate one boost control cycle and return the demanded vane position.
// Below cfg.boostMinPsi: hold cfg.spoolPercent (BPR is unreliable). Above it:
// PI loop driving BPR (= tipGaugePsi / boostGaugePsi) to cfg.bprTarget. Caller
// invokes this only when NOT in manual mode and NOT braking, so no mode/active
// flag is needed. Mutates state.
uint8_t boostBprStep(const BoostInputs &in, const BoostConfig &cfg,
                     BoostState &state, float dtSeconds);

#endif
