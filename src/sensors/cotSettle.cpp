#include "cotSettle.h"
#include <math.h>

void cotSettleInit(CotSettleState &s) {
    s.initialized = false;
    s.lastCotC = 0.0f;
    s.lastBoostPsi = 0.0f;
    s.settledTimer = 0.0f;
    s.armed = false;
    s.boostWasFlat = false;
    s.cotMoved = false;
    s.measElapsed = 0.0f;
    s.cotStart = 0.0f;
    s.count = 0;
}

// First crossing of the 63.2% point, linearly interpolated -> time constant.
static float extractTau(const CotSettleState &s, float cotStart, float cotEnd) {
    float step = cotEnd - cotStart;
    float target = cotStart + 0.632f * step;
    float tau = s.measElapsed;  // fallback
    for (uint16_t i = 0; i < s.count; i++) {
        bool reached = (step > 0.0f) ? (s.bufCot[i] >= target) : (s.bufCot[i] <= target);
        if (reached) {
            if (i == 0) {
                tau = s.bufElapsed[0];
            } else {
                float c0 = s.bufCot[i - 1], c1 = s.bufCot[i];
                float t0 = s.bufElapsed[i - 1], t1 = s.bufElapsed[i];
                float frac = (c1 != c0) ? (target - c0) / (c1 - c0) : 0.0f;
                tau = t0 + frac * (t1 - t0);
            }
            break;
        }
    }
    return tau;
}

CotSettleResult cotSettleStep(CotSettleState &s, const CotSettleConfig &cfg,
                              float cotC, float boostPsi, float dtSeconds) {
    CotSettleResult r = {false, false, 0.0f, 0.0f, 0.0f};

    if (!s.initialized) {              // seed sample: no slope available yet
        s.initialized = true;
        s.lastCotC = cotC;
        s.lastBoostPsi = boostPsi;
        s.boostWasFlat = false;
        return r;
    }

    if (dtSeconds <= 0.0f) dtSeconds = 0.0001f;
    float cotSlope = (cotC - s.lastCotC) / dtSeconds;
    float boostSlope = (boostPsi - s.lastBoostPsi) / dtSeconds;
    bool cotFlat = fabsf(cotSlope) < cfg.cotSlopeFlatCperS;
    bool boostFlat = fabsf(boostSlope) < cfg.boostSlopeFlatPsiPerS;

    // CE settled flag: both flat, sustained.
    if (cotFlat && boostFlat) {
        s.settledTimer += dtSeconds;
    } else {
        s.settledTimer = 0.0f;
    }
    r.settled = (s.settledTimer >= cfg.settledSeconds);

    // tau measurement event machine (boost-settle is the fast reference).
    if (!s.armed) {
        if (boostFlat && !s.boostWasFlat) {   // boost just went flat -> arm
            s.armed = true;
            s.cotMoved = false;
            s.measElapsed = 0.0f;
            s.cotStart = cotC;
            s.bufElapsed[0] = 0.0f;
            s.bufCot[0] = cotC;
            s.count = 1;
        }
    } else {
        s.measElapsed += dtSeconds;
        if (!boostFlat) {
            s.armed = false;                  // boost moved again -> not a clean step
        } else {
            if (s.count < cfg.maxBufferSamples && s.count < COT_SETTLE_BUFFER) {
                s.bufElapsed[s.count] = s.measElapsed;
                s.bufCot[s.count] = cotC;
                s.count++;
            } else {
                s.armed = false;              // buffer full -> abort
            }
            if (!cotFlat) s.cotMoved = true;
            if (s.armed && cotFlat && s.cotMoved) {
                float step = cotC - s.cotStart;
                if (fabsf(step) >= cfg.minStepC) {
                    r.measurementReady = true;
                    r.tauSeconds = extractTau(s, s.cotStart, cotC);
                    r.settleSeconds = s.measElapsed;
                    r.stepC = step;
                }
                s.armed = false;              // completed or discarded (too small)
            }
        }
    }

    s.boostWasFlat = boostFlat;
    s.lastCotC = cotC;
    s.lastBoostPsi = boostPsi;
    return r;
}
