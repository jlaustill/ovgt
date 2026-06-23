#ifndef cotSettle_h
#define cotSettle_h

#include <stdint.h>

struct CotSettleConfig {
    float    cotSlopeFlatCperS;     // |dCOT/dt| below this => COT "not moving"
    float    boostSlopeFlatPsiPerS; // |dBoost/dt| below this => boost "steady"
    float    settledSeconds;        // sustained-flat duration for the CE settled flag
    float    minStepC;              // min |COT change| to accept a tau measurement
    uint16_t maxBufferSamples;      // cap on samples buffered during one measurement
    float    slopeWindowSeconds;    // trailing window for the SETTLED-flag flatness
                                    // slope (0 => adjacent-sample, the legacy behavior)
};

struct CotSettleResult {
    bool  settled;          // CE-trust flag: COT + boost both flat, sustained
    bool  measurementReady; // true the one sample a tau measurement completes
    float tauSeconds;       // valid when measurementReady (63% time constant)
    float settleSeconds;    // valid when measurementReady (time to slope-flat)
    float stepC;            // signed COT change (valid when measurementReady)
    float cotSlopeCperS;    // per-step COT slope (telemetry/replay; 0 on seed sample)
    float boostSlopePsiPerS;// per-step boost slope (telemetry/replay; 0 on seed sample)
    float settleTimerS;     // accumulated flat time toward the settled flag
};

// Max samples buffered during one measurement. ~512 @ ~10 Hz covers ~51 s.
static const uint16_t COT_SETTLE_BUFFER = 512;

// Ring buffer for the settled-flag's windowed flatness slope. ~16 @ ~10 Hz covers
// ~1.6 s, comfortably more than any sane slopeWindowSeconds.
static const uint8_t COT_SLOPE_WINDOW = 16;

struct CotSettleState {
    bool     initialized;
    float    lastCotC;
    float    lastBoostPsi;
    float    settledTimer;   // accumulated flat time (CE settled flag)
    bool     armed;          // currently timing a COT settle
    bool     boostWasFlat;   // previous-sample boost-flat (edge detect)
    bool     cotMoved;       // COT left the flat band during this measurement
    float    measElapsed;    // time since arm
    float    cotStart;       // COT at arm
    uint16_t count;          // samples buffered this measurement
    float    bufElapsed[COT_SETTLE_BUFFER];
    float    bufCot[COT_SETTLE_BUFFER];
    // Trailing window of recent samples for the settled-flag flatness slope. Keeps
    // single-sample sensor noise from resetting the settled timer (the tau event
    // machine still uses the raw adjacent-sample slope, below).
    uint8_t  slopeCount;     // valid entries (saturates at COT_SLOPE_WINDOW)
    uint8_t  slopeHead;      // next write index (ring)
    float    slopeDt[COT_SLOPE_WINDOW];
    float    slopeCot[COT_SLOPE_WINDOW];
    float    slopeBoost[COT_SLOPE_WINDOW];
};

void cotSettleInit(CotSettleState &state);

// Feed one fresh COT sample (~10 Hz). Mutates state; returns the settled flag and
// (occasionally) a completed tau measurement.
CotSettleResult cotSettleStep(CotSettleState &state, const CotSettleConfig &cfg,
                              float cotC, float boostPsi, float dtSeconds);

#endif
