#include "boostBprLogic.h"

static uint8_t clampVane(float vane, uint8_t lo, uint8_t hi) {
    if (vane < (float)lo) return lo;
    if (vane > (float)hi) return hi;
    return (uint8_t)(vane + 0.5f);
}

uint8_t vaneOpenCapForBoost(float boostPsi, const BoostConfig &cfg) {
    if (cfg.vaneCapSchedule == nullptr || cfg.vaneCapScheduleLen == 0) {
        return cfg.vaneOpenCapPercent;
    }
    const VaneCapPoint *pts = cfg.vaneCapSchedule;
    uint8_t n = cfg.vaneCapScheduleLen;
    if (boostPsi <= pts[0].boostPsi) return pts[0].capPercent;
    if (boostPsi >= pts[n - 1].boostPsi) return pts[n - 1].capPercent;
    for (uint8_t i = 1; i < n; i++) {
        if (boostPsi <= pts[i].boostPsi) {
            float pLow = pts[i - 1].boostPsi;
            float pHigh = pts[i].boostPsi;
            float cLow = (float)pts[i - 1].capPercent;
            float cHigh = (float)pts[i].capPercent;
            float cap = cLow + (boostPsi - pLow) / (pHigh - pLow) * (cHigh - cLow);
            return (uint8_t)(cap + 0.5f);
        }
    }
    return pts[n - 1].capPercent;  // unreachable; satisfies the compiler
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

    // 5. Spool protection. During spool-up drive pressure leads boost, so BPR
    //    (= drive/boost) reads transiently huge and the proportional term above
    //    would fling the vane wide open, dumping the drive pressure the turbo just
    //    built and stalling the spool (the low-load boost "stick"). Below
    //    spoolProtectBoostPsi the PI may still CLOSE the vane (add drive) but may
    //    not OPEN it past the spool position. The clamp releases once boost is high
    //    enough for BPR to be a trustworthy control signal (BPR is back near target
    //    by then, so the handoff is smooth).
    if (in.boostGaugePsi < cfg.spoolProtectBoostPsi && vane > (float)cfg.spoolPercent) {
        vane = (float)cfg.spoolPercent;
    }

    // 6. Bound the PI demand to [spoolPercent, vaneOpenCapPercent]. The cap blunts
    //    the open-slam; the floor — the SAME spoolPercent used to hold position
    //    during spool — stops the loop from slamming fully closed, so even a
    //    high-load limit cycle can only swing between the spool position and the
    //    cap (e.g. 22<->55), never stop-to-stop. Tuning spoolPercent moves both the
    //    spool hold and this floor together.
    return clampVane(vane, cfg.spoolPercent, cfg.vaneOpenCapPercent);
}

