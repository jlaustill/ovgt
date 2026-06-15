#include <unity.h>
#include "control/boostBprLogic.h"

void setUp(void) {}
void tearDown(void) {}

// Matches the production config defaults (see boostController.cpp). NOTE: kp is
// large (~full vane travel per unit BPR error) because BPR error is a 0..1 ratio,
// not psi.
static BoostConfig makeConfig(void) {
    BoostConfig cfg;
    cfg.bprTarget = 1.0f;
    cfg.boostMinPsi = 2.0f;
    cfg.spoolPercent = 25;
    cfg.vaneClosedPercent = 0;
    cfg.vaneOpenPercent = 88;
    cfg.kp = 88.0f;
    cfg.ki = 20.0f;
    return cfg;
}

// Below the boost threshold, BPR is garbage (divide by ~0): hold the spool
// position and remember we were spooling.
void test_spool_region_holds_spool_position(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 1.0f;  // < boostMinPsi (2.0)
    in.tipGaugePsi = 0.5f;
    BoostState st = {0.0f, false};
    uint8_t vane = boostBprStep(in, cfg, st, 0.01f);
    TEST_ASSERT_EQUAL_UINT8(25, vane);
    TEST_ASSERT_TRUE(st.wasSpooling);
}

// BPR below target (drive too low) -> close vanes to raise drive pressure.
void test_bpr_below_target_closes(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 20.0f;
    in.tipGaugePsi = 10.0f;  // bpr = 0.5, error = 0.5
    BoostState st = {0.0f, false};
    uint8_t vane = boostBprStep(in, cfg, st, 0.01f);
    // integral = 20*0.5*0.01 = 0.1; closure = 88*0.5 + 0.1 = 44.1
    // vane = open(88) - 44.1 = 43.9 -> 44
    TEST_ASSERT_EQUAL_UINT8(44, vane);
    TEST_ASSERT_TRUE(vane < cfg.vaneOpenPercent);  // closing, not opening
}

// BPR above target (drive too high) -> open vanes (clamps fully open).
void test_bpr_above_target_opens(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 20.0f;
    in.tipGaugePsi = 30.0f;  // bpr = 1.5, error = -0.5
    BoostState st = {0.0f, false};
    uint8_t vane = boostBprStep(in, cfg, st, 0.01f);
    // negative error -> integral clamps to 0, closure = -44 -> vane clamps to open
    TEST_ASSERT_EQUAL_UINT8(88, vane);
}

// A bigger BPR deficit closes the vanes more.
void test_bigger_bpr_deficit_closes_more(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs small;
    small.boostGaugePsi = 20.0f;
    small.tipGaugePsi = 18.0f;  // bpr = 0.9, error = 0.1
    BoostState stSmall = {0.0f, false};
    uint8_t vaneSmall = boostBprStep(small, cfg, stSmall, 0.01f);

    BoostInputs big;
    big.boostGaugePsi = 20.0f;
    big.tipGaugePsi = 10.0f;   // bpr = 0.5, error = 0.5
    BoostState stBig = {0.0f, false};
    uint8_t vaneBig = boostBprStep(big, cfg, stBig, 0.01f);

    TEST_ASSERT_TRUE(vaneBig < vaneSmall);  // more deficit => more closure => lower %
}

// Handoff from spool to PI starts at the spool position, NOT a jump to fully open.
void test_bumpless_handoff_from_spool(void) {
    BoostConfig cfg = makeConfig();
    BoostState st = {0.0f, false};

    BoostInputs spool;
    spool.boostGaugePsi = 1.0f;  // spool region
    spool.tipGaugePsi = 20.0f;
    uint8_t vaneSpool = boostBprStep(spool, cfg, st, 0.01f);
    TEST_ASSERT_EQUAL_UINT8(25, vaneSpool);

    BoostInputs handoff;
    handoff.boostGaugePsi = 20.0f;  // now above threshold
    handoff.tipGaugePsi = 20.0f;    // bpr = 1.0, error = 0
    uint8_t vaneHandoff = boostBprStep(handoff, cfg, st, 0.01f);
    // seeded integral = open(88) - spool(25) = 63; closure = 88*0 + 63 = 63
    // vane = 88 - 63 = 25  (continues from spool, no jump to 88)
    TEST_ASSERT_EQUAL_UINT8(25, vaneHandoff);
    TEST_ASSERT_TRUE(vaneHandoff < cfg.vaneOpenPercent);
}

// Persistent large deficit: integral winds up to full travel, vanes fully closed.
void test_integral_antiwindup_clamps_closed(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 20.0f;
    in.tipGaugePsi = 0.0f;  // bpr = 0, error = 1.0 (persistent)
    BoostState st = {0.0f, false};
    uint8_t vane = 0;
    for (int i = 0; i < 1000; i++) {
        vane = boostBprStep(in, cfg, st, 0.01f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 88.0f, st.integralTerm);  // clamped at full travel
    TEST_ASSERT_EQUAL_UINT8(0, vane);                          // fully closed
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_spool_region_holds_spool_position);
    RUN_TEST(test_bpr_below_target_closes);
    RUN_TEST(test_bpr_above_target_opens);
    RUN_TEST(test_bigger_bpr_deficit_closes_more);
    RUN_TEST(test_bumpless_handoff_from_spool);
    RUN_TEST(test_integral_antiwindup_clamps_closed);
    return UNITY_END();
}
