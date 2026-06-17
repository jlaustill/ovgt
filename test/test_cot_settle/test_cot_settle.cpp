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

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_settled_flag_after_sustained_flat);
    RUN_TEST(test_settled_resets_on_spike);
    RUN_TEST(test_tau_extraction_first_order);
    RUN_TEST(test_no_measurement_when_boost_moving);
    RUN_TEST(test_small_step_rejected);
    return UNITY_END();
}
