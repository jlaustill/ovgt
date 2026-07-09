#include <unity.h>
#include <math.h>
#include "sensors/cotSettle.h"

void setUp(void) {}
void tearDown(void) {}

static CotSettleConfig makeConfig(void) {
    CotSettleConfig cfg;
    cfg.cotSlopeFlatCperS = 0.3f;
    cfg.boostSlopeFlatPsiPerS = 0.5f;
    cfg.settledSeconds = 2.0f;
    cfg.minStepC = 3.0f;
    cfg.maxBufferSamples = COT_SETTLE_BUFFER;
    cfg.slopeWindowSeconds = 0.5f;
    return cfg;
}

// Sustained flat COT + flat boost -> settled after settledSeconds.
void test_settled_flag_after_sustained_flat(void) {
    CotSettleConfig cfg = makeConfig();
    CotSettleState st; cotSettleInit(st);
    cotSettleStep(st, cfg, 50.0f, 5.0f, 0.1f);  // seed
    CotSettleResult r = {false, false, 0, 0, 0};
    for (int k = 0; k < 25; k++) r = cotSettleStep(st, cfg, 50.0f, 5.0f, 0.1f);
    TEST_ASSERT_TRUE(r.settled);  // 25*0.1 = 2.5s >= 2.0s
}

// REGRESSION (settled-flag never clears, 0/13394 on drive-2): a steady engine with
// realistic K-type per-sample noise (~+/-0.08 C jitter at ~10 Hz => adjacent-sample
// slope swings +/-0.8..1.6 C/s, far above the 0.3 C/s flat threshold) must STILL be
// recognized as settled. Adjacent-sample flatness never settles here (the bug); a
// windowed-slope flatness decision does.
void test_settled_flag_with_realistic_cot_noise(void) {
    CotSettleConfig cfg = makeConfig();
    CotSettleState st; cotSettleInit(st);
    // Deterministic zero-mean jitter sequence, |adjacent slope| up to ~1.6 C/s.
    const float jitter[] = {0.06f, -0.04f, 0.08f, -0.06f, 0.02f, -0.08f, 0.04f, -0.02f};
    const int n = sizeof(jitter) / sizeof(jitter[0]);
    cotSettleStep(st, cfg, 50.0f, 5.0f, 0.1f);  // seed
    bool everSettled = false;
    for (int k = 0; k < 60; k++) {              // 6 s of steady-but-noisy idle
        float cot = 50.0f + jitter[k % n];
        CotSettleResult r = cotSettleStep(st, cfg, cot, 5.0f, 0.1f);
        everSettled = everSettled || r.settled;
    }
    TEST_ASSERT_TRUE(everSettled);
}

// A COT spike after being settled clears the flag.
void test_settled_resets_on_spike(void) {
    CotSettleConfig cfg = makeConfig();
    CotSettleState st; cotSettleInit(st);
    cotSettleStep(st, cfg, 50.0f, 5.0f, 0.1f);
    for (int k = 0; k < 25; k++) cotSettleStep(st, cfg, 50.0f, 5.0f, 0.1f);
    CotSettleResult r = cotSettleStep(st, cfg, 60.0f, 5.0f, 0.1f);  // +100 C/s spike
    TEST_ASSERT_FALSE(r.settled);
}

// Boost flat, COT follows a known first-order step -> tau extracted ~correctly.
void test_tau_extraction_first_order(void) {
    CotSettleConfig cfg = makeConfig();
    CotSettleState st; cotSettleInit(st);
    const float dt = 0.1f, tau = 0.5f, c0 = 20.0f, c1 = 120.0f;
    cotSettleStep(st, cfg, c0, 0.0f, dt);   // seed
    cotSettleStep(st, cfg, c0, 2.0f, dt);   // boost moving (slope 20) -> not flat
    CotSettleResult r = {false, false, 0, 0, 0};
    float t = 0.0f;
    for (int k = 0; k < 200 && !r.measurementReady; k++) {
        float cot = c1 - (c1 - c0) * expf(-t / tau);  // arm at t=0 (boost now flat)
        r = cotSettleStep(st, cfg, cot, 2.0f, dt);
        t += dt;
    }
    TEST_ASSERT_TRUE(r.measurementReady);
    TEST_ASSERT_FLOAT_WITHIN(0.08f, 0.5f, r.tauSeconds);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 100.0f, r.stepC);
}

// Boost never settles -> never arms -> no measurement.
void test_no_measurement_when_boost_moving(void) {
    CotSettleConfig cfg = makeConfig();
    CotSettleState st; cotSettleInit(st);
    cotSettleStep(st, cfg, 20.0f, 0.0f, 0.1f);  // seed
    bool any = false;
    for (int k = 1; k < 40; k++) {
        float cot = 120.0f - 100.0f * expf(-(k * 0.1f) / 0.5f);
        float boost = (float)k * 2.0f;  // always rising -> slope 20, never flat
        CotSettleResult r = cotSettleStep(st, cfg, cot, boost, 0.1f);
        any = any || r.measurementReady;
    }
    TEST_ASSERT_FALSE(any);
}

// A clean settle but with a tiny COT step (< minStepC) is discarded.
void test_small_step_rejected(void) {
    CotSettleConfig cfg = makeConfig();
    CotSettleState st; cotSettleInit(st);
    cotSettleStep(st, cfg, 20.0f, 0.0f, 0.1f);  // seed
    cotSettleStep(st, cfg, 20.0f, 2.0f, 0.1f);  // boost moving
    // boost flat; COT nudges +1 C then holds (well under minStepC=3)
    float cots[] = {20.0f, 20.6f, 21.0f, 21.0f, 21.0f, 21.0f, 21.0f, 21.0f};
    bool any = false;
    for (unsigned i = 0; i < sizeof(cots) / sizeof(cots[0]); i++) {
        CotSettleResult r = cotSettleStep(st, cfg, cots[i], 2.0f, 0.1f);
        any = any || r.measurementReady;
    }
    TEST_ASSERT_FALSE(any);
}

// Per-step slopes and the settle timer are surfaced for telemetry/replay.
void test_surfaced_slopes_and_timer(void) {
    CotSettleConfig cfg = makeConfig();
    CotSettleState st; cotSettleInit(st);
    cotSettleStep(st, cfg, 50.0f, 5.0f, 0.1f);  // seed: no slope yet
    // One moving sample: dCOT=+1 C over 0.1 s => 10 C/s; dBoost=+0.5 psi => 5 psi/s.
    CotSettleResult moving = cotSettleStep(st, cfg, 51.0f, 5.5f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, moving.cotSlopeCperS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.0f, moving.boostSlopePsiPerS);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, moving.settleTimerS);  // not flat -> reset
    // Hold flat. Surfaced slopes are the raw adjacent values (~0 once holding). The
    // settle timer uses the WINDOWED flatness, so after the +1 C step it only begins
    // accumulating once that step rolls out of the 0.5 s slope window (~6 samples);
    // hold long enough and the timer is clearly accumulating.
    CotSettleResult flat = {false, false, 0, 0, 0};
    for (int k = 0; k < 12; k++) flat = cotSettleStep(st, cfg, 51.0f, 5.5f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, flat.cotSlopeCperS);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, flat.boostSlopePsiPerS);
    TEST_ASSERT_TRUE(flat.settleTimerS > 0.3f);  // window cleared the step, timer running
}

// At the native ADS rate (~200 Hz, dt = 0.005 s) a sustained-flat COT + boost must
// still settle: the 0.5 s windowed-slope flatness needs the slope ring to hold a
// full 0.5 s (>= 100 samples), which only passes with the grown COT_SLOPE_WINDOW.
void test_settled_flag_at_ads_rate(void) {
    CotSettleConfig cfg = makeConfig();
    CotSettleState st; cotSettleInit(st);
    cotSettleStep(st, cfg, 50.0f, 5.0f, 0.005f);  // seed
    CotSettleResult r = {false, false, 0, 0, 0};
    for (int k = 0; k < 500; k++) r = cotSettleStep(st, cfg, 50.0f, 5.0f, 0.005f);
    TEST_ASSERT_TRUE(r.settled);  // 500*0.005 = 2.5 s >= 2.0 s
    TEST_ASSERT_TRUE(COT_SLOPE_WINDOW >= 100);  // ring must span 0.5 s at 200 Hz
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_settled_flag_after_sustained_flat);
    RUN_TEST(test_settled_flag_at_ads_rate);
    RUN_TEST(test_settled_flag_with_realistic_cot_noise);
    RUN_TEST(test_settled_resets_on_spike);
    RUN_TEST(test_tau_extraction_first_order);
    RUN_TEST(test_no_measurement_when_boost_moving);
    RUN_TEST(test_small_step_rejected);
    RUN_TEST(test_surfaced_slopes_and_timer);
    return UNITY_END();
}
