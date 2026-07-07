#ifndef boostBprLogic_h
#define boostBprLogic_h

#include <stdint.h>

struct BoostInputs {
    float boostGaugePsi;   // (COP - CIP) in psi
    float tipGaugePsi;     // turbine inlet (drive) pressure, gauge psi
};

struct VaneCapPoint {
    float   boostPsi;     // schedule breakpoint (ascending)
    uint8_t capPercent;   // open-cap at/above this boost (until the next point)
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
    // Boost -> open-cap schedule (ascending boostPsi). NULL / len 0 => use the flat
    // vaneOpenCapPercent above. Tightens open authority under load so the PI cannot
    // slam the vane wide open. Floor stays spoolPercent.
    const VaneCapPoint *vaneCapSchedule;
    uint8_t             vaneCapScheduleLen;
};

struct BoostState {
    float integralTerm;  // accumulated integral CONTRIBUTION, in vane-% units
    bool  wasSpooling;   // pending spool->PI handoff seed (one-shot)
    bool  inPiRegion;    // hysteretic region: false = spool, true = PI
    uint8_t lastVaneCap; // open-cap used on the last step (for telemetry readout)
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

// Boost (gauge psi) -> vane open-cap %, linear between ascending schedule points,
// flat outside the ends. Falls back to cfg.vaneOpenCapPercent when no schedule is
// wired (cfg.vaneCapSchedule == NULL or cfg.vaneCapScheduleLen == 0).
uint8_t vaneOpenCapForBoost(float boostPsi, const BoostConfig &cfg);

#endif
