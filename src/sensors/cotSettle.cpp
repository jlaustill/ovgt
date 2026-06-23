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
    s.slopeCount = 0;
    s.slopeHead = 0;
}

// Push one sample into the trailing slope-window ring buffer.
static void pushSlopeSample(CotSettleState &s, float dt, float cotC, float boostPsi) {
    s.slopeDt[s.slopeHead] = dt;
    s.slopeCot[s.slopeHead] = cotC;
    s.slopeBoost[s.slopeHead] = boostPsi;
    s.slopeHead = (uint8_t)((s.slopeHead + 1) % COT_SLOPE_WINDOW);
    if (s.slopeCount < COT_SLOPE_WINDOW) s.slopeCount++;
}

// Least-squares slope of COT and boost over the trailing window (newest sample at
// t=0, older samples at negative times). Robust to single-sample sensor noise that
// the adjacent-sample slope amplifies. Falls back to the adjacent slopes when fewer
// than two points span any time. `window <= 0` disables windowing (legacy behavior).
static void windowedSlopes(const CotSettleState &s, float window,
                           float adjCotSlope, float adjBoostSlope,
                           float &cotSlopeOut, float &boostSlopeOut) {
    cotSlopeOut = adjCotSlope;
    boostSlopeOut = adjBoostSlope;
    if (window <= 0.0f || s.slopeCount < 2) return;

    float t[COT_SLOPE_WINDOW], c[COT_SLOPE_WINDOW], b[COT_SLOPE_WINDOW];
    int m = 0;
    float curT = 0.0f;
    for (uint8_t k = 0; k < s.slopeCount; k++) {
        uint8_t idx = (uint8_t)((s.slopeHead + COT_SLOPE_WINDOW - 1 - k) % COT_SLOPE_WINDOW);
        if (k > 0) {
            uint8_t newer = (uint8_t)((s.slopeHead + COT_SLOPE_WINDOW - k) % COT_SLOPE_WINDOW);
            curT -= s.slopeDt[newer];  // dt is stored with the newer sample
        }
        t[m] = curT; c[m] = s.slopeCot[idx]; b[m] = s.slopeBoost[idx];
        m++;
        if (-curT >= window) break;    // window covered (this sample brackets the edge)
    }
    if (m < 2) return;

    float tBar = 0.0f, cBar = 0.0f, bBar = 0.0f;
    for (int i = 0; i < m; i++) { tBar += t[i]; cBar += c[i]; bBar += b[i]; }
    tBar /= m; cBar /= m; bBar /= m;
    float stt = 0.0f, stc = 0.0f, stb = 0.0f;
    for (int i = 0; i < m; i++) {
        float dT = t[i] - tBar;
        stt += dT * dT;
        stc += dT * (c[i] - cBar);
        stb += dT * (b[i] - bBar);
    }
    if (stt <= 1e-9f) return;          // no time span -> keep adjacent fallback
    cotSlopeOut = stc / stt;
    boostSlopeOut = stb / stt;
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

    if (dtSeconds <= 0.0f) dtSeconds = 0.0001f;

    if (!s.initialized) {              // seed sample: no slope available yet
        s.initialized = true;
        s.lastCotC = cotC;
        s.lastBoostPsi = boostPsi;
        s.boostWasFlat = false;
        pushSlopeSample(s, dtSeconds, cotC, boostPsi);
        return r;
    }

    // Adjacent-sample slopes: surfaced for telemetry/replay AND used by the tau event
    // machine (which keys off a clean boost step, so raw slope is correct there).
    float cotSlope = (cotC - s.lastCotC) / dtSeconds;
    float boostSlope = (boostPsi - s.lastBoostPsi) / dtSeconds;
    bool cotFlat = fabsf(cotSlope) < cfg.cotSlopeFlatCperS;
    bool boostFlat = fabsf(boostSlope) < cfg.boostSlopeFlatPsiPerS;

    // Settled flag uses a WINDOWED flatness so single-sample COT sensor noise (which
    // the adjacent slope amplifies to ~0.4-1.5 C/s at idle, above the 0.3 C/s
    // threshold) cannot keep resetting the sustained-flat timer.
    pushSlopeSample(s, dtSeconds, cotC, boostPsi);
    float cotSlopeWin, boostSlopeWin;
    windowedSlopes(s, cfg.slopeWindowSeconds, cotSlope, boostSlope,
                   cotSlopeWin, boostSlopeWin);
    bool cotFlatWin = fabsf(cotSlopeWin) < cfg.cotSlopeFlatCperS;
    bool boostFlatWin = fabsf(boostSlopeWin) < cfg.boostSlopeFlatPsiPerS;

    // CE settled flag: both flat (windowed), sustained.
    if (cotFlatWin && boostFlatWin) {
        s.settledTimer += dtSeconds;
    } else {
        s.settledTimer = 0.0f;
    }
    r.settled = (s.settledTimer >= cfg.settledSeconds);
    r.cotSlopeCperS = cotSlope;          // surfaced = raw instantaneous slope
    r.boostSlopePsiPerS = boostSlope;
    r.settleTimerS = s.settledTimer;

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
