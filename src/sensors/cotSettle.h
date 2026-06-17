#ifndef cotSettle_h
#define cotSettle_h

#include <stdint.h>

struct CotSettleConfig {
    float    cotSlopeFlatCperS;     // |dCOT/dt| below this => COT "not moving"
    float    boostSlopeFlatPsiPerS; // |dBoost/dt| below this => boost "steady"
    float    settledSeconds;        // sustained-flat duration for the CE settled flag
    float    minStepC;              // min |COT change| to accept a tau measurement
    uint16_t maxBufferSamples;      // cap on samples buffered during one measurement
};

struct CotSettleResult {
    bool  settled;          // CE-trust flag: COT + boost both flat, sustained
    bool  measurementReady; // true the one sample a tau measurement completes
    float tauSeconds;       // valid when measurementReady (63% time constant)
    float settleSeconds;    // valid when measurementReady (time to slope-flat)
    float stepC;            // signed COT change (valid when measurementReady)
};

// Max samples buffered during one measurement. ~512 @ ~10 Hz covers ~51 s.
static const uint16_t COT_SETTLE_BUFFER = 512;

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
};

void cotSettleInit(CotSettleState &state);

// Feed one fresh COT sample (~10 Hz). Mutates state; returns the settled flag and
// (occasionally) a completed tau measurement.
CotSettleResult cotSettleStep(CotSettleState &state, const CotSettleConfig &cfg,
                              float cotC, float boostPsi, float dtSeconds);

#endif
