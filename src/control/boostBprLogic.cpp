#include "boostBprLogic.h"

static uint8_t clampVane(float vane, uint8_t lo, uint8_t hi) {
    if (vane < (float)lo) return lo;
    if (vane > (float)hi) return hi;
    return (uint8_t)(vane + 0.5f);
}

// Vane convention: vaneClosedPercent (0) = fully closed = max drive pressure /
// boost; vaneOpenPercent (88) = fully open = no restriction. BPR = drive / boost.
// Closing vanes raises drive faster than boost, so BPR rises. To hold BPR at
// target: BPR below target -> close (raise drive); BPR above target -> open.
uint8_t boostBprStep(const BoostInputs &in, const BoostConfig &cfg,
                     BoostState &state, float dtSeconds) {
    // 1. Region selection with hysteresis. A single boost threshold made the vane
    //    chatter (spool<->PI) when boost danced across it at light load. Two
    //    thresholds give a dead band: climb into PI above boostPiPsi, drop back to
    //    spool below boostSpoolPsi, hold the current region in between.
    if (state.inPiRegion) {
        if (in.boostGaugePsi < cfg.boostSpoolPsi) state.inPiRegion = false;
    } else {
        if (in.boostGaugePsi > cfg.boostPiPsi) state.inPiRegion = true;
    }

    // 2. Spool region. BPR = drive/boost is unreliable at low boost (divide by
    //    ~0), and a runaway BPR would fling the vanes open — the opposite of what
    //    spool needs. Hold a fixed spool position and flag the pending handoff.
    if (!state.inPiRegion) {
        state.wasSpooling = true;
        return clampVane((float)cfg.spoolPercent,
                         cfg.vaneClosedPercent, cfg.vaneOpenPercent);
    }

    float travel = (float)cfg.vaneOpenPercent - (float)cfg.vaneClosedPercent;

    // 3. Bumpless handoff. On the first PI cycle after spooling, seed the integral
    //    so the PI's output starts AT the spool position instead of jumping to
    //    fully open (vane = open - 0). Then the loop adjusts from there.
    if (state.wasSpooling) {
        state.integralTerm = (float)cfg.vaneOpenPercent - (float)cfg.spoolPercent;
        state.wasSpooling = false;
    }

    // 4. PI loop on BPR. No derivative term: BPR is a noisy ratio. Positive error
    //    (BPR below target) means we need MORE drive pressure, i.e. MORE closure,
    //    i.e. a LOWER vane position. Closure adds up and subtracts from open. The
    //    integral is clamped to [0, travel] (anti-windup); a position-tracking gate
    //    was tried and reverted — it starved the integral during normal slewing
    //    and left the vane too open (lost low-RPM boost).
    float bpr = in.tipGaugePsi / in.boostGaugePsi;
    float error = cfg.bprTarget - bpr;
    state.integralTerm += cfg.ki * error * dtSeconds;
    if (state.integralTerm < 0.0f) {
        state.integralTerm = 0.0f;
    }
    if (state.integralTerm > travel) {
        state.integralTerm = travel;  // cannot exceed full travel
    }
    float closure = cfg.kp * error + state.integralTerm;
    float vane = (float)cfg.vaneOpenPercent - closure;
    return clampVane(vane, cfg.vaneClosedPercent, cfg.vaneOpenPercent);
}
