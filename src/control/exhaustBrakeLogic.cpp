#include "exhaustBrakeLogic.h"

static uint8_t clampVane(float vane, uint8_t lo, uint8_t hi) {
    if (vane < (float)lo) return lo;
    if (vane > (float)hi) return hi;
    return (uint8_t)(vane + 0.5f);
}

BrakeOutput exhaustBrakeStep(const BrakeInputs &in, const BrakeConfig &cfg,
                             BrakeState &state, float dtSeconds) {
    BrakeOutput out;
    out.active = false;
    out.vanePercent = cfg.minVanePercent;
    out.fault = false;

    // 1. Manual override yields to the existing serial/boost behavior.
    if (in.manualMode) {
        state.integralTerm = 0.0f;
        return out;
    }

    // 2. Stale-CAN failsafe: never act on stale lockup/throttle/load.
    bool eec2Stale = (in.nowMs - in.lastEec2RxMs) > cfg.staleTimeoutMs;
    bool etc1Stale = (in.nowMs - in.lastEtc1RxMs) > cfg.staleTimeoutMs;
    if (eec2Stale || etc1Stale) {
        state.integralTerm = 0.0f;
        return out;
    }

    // 3. Instant release: the driver always wins.
    if (in.acceleratorPedalPercent > 0 || in.engineLoadPercent > 0) {
        state.integralTerm = 0.0f;
        return out;
    }

    // 4. Engage condition: torque converter locked (status 0b01) and no power.
    bool locked = (in.torqueConverterLockupStatus == 0x01);
    if (!locked) {
        state.integralTerm = 0.0f;
        return out;
    }

    out.active = true;

    // 5. Ceiling cutoff (hardware protection against valve float): force vanes
    //    open, flag a fault, and hold the integral.
    if (in.tipGaugePsi > cfg.ceilingPsi) {
        out.fault = true;
        out.vanePercent = cfg.minVanePercent;
        return out;
    }

    // 6. PI loop. No derivative term: TIP is noisy and D would jitter the vanes.
    float error = cfg.targetPsi - in.tipGaugePsi;
    state.integralTerm += cfg.ki * error * dtSeconds;
    if (state.integralTerm < 0.0f) {
        state.integralTerm = 0.0f;
    }
    if (state.integralTerm > (float)cfg.maxVanePercent) {
        state.integralTerm = (float)cfg.maxVanePercent;  // anti-windup
    }
    float vane = cfg.kp * error + state.integralTerm;
    out.vanePercent = clampVane(vane, cfg.minVanePercent, cfg.maxVanePercent);
    return out;
}
