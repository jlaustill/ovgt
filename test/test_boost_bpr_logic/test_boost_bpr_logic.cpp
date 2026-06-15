#include <unity.h>
#include "control/boostBprLogic.h"

void setUp(void) {}
void tearDown(void) {}

// Matches the production config defaults (see boostController.cpp). NOTE: kp/ki
// are tuned for a BPR error that is a 0..1 ratio, not psi.
static BoostConfig makeConfig(void) {
    BoostConfig cfg;
    cfg.bprTarget = 1.0f;
    cfg.boostSpoolPsi = 1.5f;
    cfg.boostPiPsi = 3.0f;
    cfg.spoolPercent = 25;
    cfg.vaneClosedPercent = 0;
    cfg.vaneOpenPercent = 88;
    cfg.kp = 20.0f;
    cfg.ki = 20.0f;
    cfg.integralTrackingBand = 8.0f;
    return cfg;
}

// Below the spool threshold, BPR is garbage (divide by ~0): hold the spool
// position and remember we were spooling.
void test_spool_region_holds_spool_position(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 1.0f;  // < boostSpoolPsi (1.5)
    in.tipGaugePsi = 0.5f;
    in.actuatorReportedPercent = 0;
    BoostState st = {0.0f, false, false, 0};
    uint8_t vane = boostBprStep(in, cfg, st, 0.01f);
    TEST_ASSERT_EQUAL_UINT8(25, vane);
    TEST_ASSERT_TRUE(st.wasSpooling);
    TEST_ASSERT_FALSE(st.inPiRegion);
}

// BPR below target (drive too low) -> close vanes to raise drive pressure.
void test_bpr_below_target_closes(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 20.0f;
    in.tipGaugePsi = 10.0f;  // bpr = 0.5, error = 0.5
    in.actuatorReportedPercent = 88;            // actuator at last command (tracking)
    BoostState st = {0.0f, false, true, 88};    // PI region, last commanded open
    uint8_t vane = boostBprStep(in, cfg, st, 0.01f);
    // integral = 20*0.5*0.01 = 0.1; closure = 20*0.5 + 0.1 = 10.1
    // vane = open(88) - 10.1 = 77.9 -> 78
    TEST_ASSERT_EQUAL_UINT8(78, vane);
    TEST_ASSERT_TRUE(vane < cfg.vaneOpenPercent);  // closing, not opening
}

// BPR above target (drive too high) -> open vanes (clamps fully open).
void test_bpr_above_target_opens(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 20.0f;
    in.tipGaugePsi = 30.0f;  // bpr = 1.5, error = -0.5
    in.actuatorReportedPercent = 88;
    BoostState st = {0.0f, false, true, 88};
    uint8_t vane = boostBprStep(in, cfg, st, 0.01f);
    // negative error -> integral clamps to 0, closure = -10 -> vane clamps to open
    TEST_ASSERT_EQUAL_UINT8(88, vane);
}

// A bigger BPR deficit closes the vanes more.
void test_bigger_bpr_deficit_closes_more(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs small;
    small.boostGaugePsi = 20.0f;
    small.tipGaugePsi = 18.0f;  // bpr = 0.9, error = 0.1
    small.actuatorReportedPercent = 88;
    BoostState stSmall = {0.0f, false, true, 88};
    uint8_t vaneSmall = boostBprStep(small, cfg, stSmall, 0.01f);

    BoostInputs big;
    big.boostGaugePsi = 20.0f;
    big.tipGaugePsi = 10.0f;   // bpr = 0.5, error = 0.5
    big.actuatorReportedPercent = 88;
    BoostState stBig = {0.0f, false, true, 88};
    uint8_t vaneBig = boostBprStep(big, cfg, stBig, 0.01f);

    TEST_ASSERT_TRUE(vaneBig < vaneSmall);  // more deficit => more closure => lower %
}

// Handoff from spool to PI starts at the spool position, NOT a jump to fully open.
void test_bumpless_handoff_from_spool(void) {
    BoostConfig cfg = makeConfig();
    BoostState st = {0.0f, false, false, 0};

    BoostInputs spool;
    spool.boostGaugePsi = 1.0f;  // spool region (< boostSpoolPsi)
    spool.tipGaugePsi = 20.0f;
    spool.actuatorReportedPercent = 25;
    uint8_t vaneSpool = boostBprStep(spool, cfg, st, 0.01f);
    TEST_ASSERT_EQUAL_UINT8(25, vaneSpool);

    BoostInputs handoff;
    handoff.boostGaugePsi = 20.0f;  // now above boostPiPsi -> PI region
    handoff.tipGaugePsi = 20.0f;    // bpr = 1.0, error = 0
    handoff.actuatorReportedPercent = 25;  // actuator still at spool (tracking)
    uint8_t vaneHandoff = boostBprStep(handoff, cfg, st, 0.01f);
    // seeded integral = open(88) - spool(25) = 63; closure = 20*0 + 63 = 63
    // vane = 88 - 63 = 25  (continues from spool, no jump to 88)
    TEST_ASSERT_EQUAL_UINT8(25, vaneHandoff);
    TEST_ASSERT_TRUE(vaneHandoff < cfg.vaneOpenPercent);
}

// Persistent large deficit with the actuator tracking: integral winds up to full
// travel and the vanes go fully closed. (Reported position follows the command
// each cycle to model a perfect actuator.)
void test_integral_antiwindup_clamps_closed(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 20.0f;
    in.tipGaugePsi = 0.0f;  // bpr = 0, error = 1.0 (persistent)
    BoostState st = {0.0f, false, true, 25};
    uint8_t vane = 25;
    for (int i = 0; i < 1000; i++) {
        in.actuatorReportedPercent = vane;  // actuator reaches last command
        vane = boostBprStep(in, cfg, st, 0.01f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 88.0f, st.integralTerm);  // clamped at full travel
    TEST_ASSERT_EQUAL_UINT8(0, vane);                          // fully closed
}

// Anti-windup core: when the actuator has NOT reached the last command (large
// tracking error), the integral must NOT accumulate, even with a standing error.
void test_integral_freezes_when_actuator_lags(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 20.0f;
    in.tipGaugePsi = 10.0f;            // bpr 0.5, error +0.5 (wants to wind closed)
    in.actuatorReportedPercent = 88;   // actuator wide open...
    BoostState st = {0.0f, false, true, 0};  // ...but we last commanded closed (0)
    // trackingError = |0 - 88| = 88 > band(8) -> integral frozen
    boostBprStep(in, cfg, st, 0.01f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, st.integralTerm);
}

// Hysteresis: once in PI, a boost dip into the dead band (between the two
// thresholds) keeps the PI region instead of chattering back to spool.
void test_hysteresis_stays_pi_in_band(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 2.0f;  // between boostSpoolPsi(1.5) and boostPiPsi(3.0)
    in.tipGaugePsi = 2.0f;
    in.actuatorReportedPercent = 33;
    BoostState st = {0.0f, false, true, 33};
    boostBprStep(in, cfg, st, 0.01f);
    TEST_ASSERT_TRUE(st.inPiRegion);  // held PI, no chatter to spool
}

// Hysteresis: below the low threshold, fall back to spool.
void test_hysteresis_drops_to_spool_below_low(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 1.0f;  // < boostSpoolPsi
    in.tipGaugePsi = 1.0f;
    in.actuatorReportedPercent = 33;
    BoostState st = {0.0f, false, true, 33};
    uint8_t vane = boostBprStep(in, cfg, st, 0.01f);
    TEST_ASSERT_FALSE(st.inPiRegion);
    TEST_ASSERT_EQUAL_UINT8(25, vane);  // spool position
}

// Hysteresis: in the dead band coming from spool, stay in spool (no premature PI).
void test_hysteresis_stays_spool_in_band(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 2.0f;  // between thresholds
    in.tipGaugePsi = 2.0f;
    in.actuatorReportedPercent = 25;
    BoostState st = {0.0f, false, false, 25};
    uint8_t vane = boostBprStep(in, cfg, st, 0.01f);
    TEST_ASSERT_FALSE(st.inPiRegion);
    TEST_ASSERT_EQUAL_UINT8(25, vane);
}

// Hysteresis: above the high threshold, engage PI.
void test_hysteresis_enters_pi_above_high(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 3.5f;  // > boostPiPsi
    in.tipGaugePsi = 3.5f;
    in.actuatorReportedPercent = 25;
    BoostState st = {0.0f, false, false, 25};
    boostBprStep(in, cfg, st, 0.01f);
    TEST_ASSERT_TRUE(st.inPiRegion);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_spool_region_holds_spool_position);
    RUN_TEST(test_bpr_below_target_closes);
    RUN_TEST(test_bpr_above_target_opens);
    RUN_TEST(test_bigger_bpr_deficit_closes_more);
    RUN_TEST(test_bumpless_handoff_from_spool);
    RUN_TEST(test_integral_antiwindup_clamps_closed);
    RUN_TEST(test_integral_freezes_when_actuator_lags);
    RUN_TEST(test_hysteresis_stays_pi_in_band);
    RUN_TEST(test_hysteresis_drops_to_spool_below_low);
    RUN_TEST(test_hysteresis_stays_spool_in_band);
    RUN_TEST(test_hysteresis_enters_pi_above_high);
    return UNITY_END();
}
