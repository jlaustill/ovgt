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
    cfg.spoolProtectBoostPsi = 6.0f;
    cfg.vaneOpenCapPercent = 55;
    return cfg;
}

// Below the spool threshold, BPR is garbage (divide by ~0): hold the spool
// position and remember we were spooling.
void test_spool_region_holds_spool_position(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 1.0f;  // < boostSpoolPsi (1.5)
    in.tipGaugePsi = 0.5f;
    BoostState st = {0.0f, false, false};
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
    BoostState st = {40.0f, false, true};  // in PI region with a wound integral
    uint8_t vane = boostBprStep(in, cfg, st, 0.01f);
    // integral 40 + 20*0.5*0.01 = 40.1; closure = 20*0.5 + 40.1 = 50.1
    // vane = open(88) - 50.1 = 37.9 -> 38 (below the open cap; genuinely closing)
    TEST_ASSERT_EQUAL_UINT8(38, vane);
    TEST_ASSERT_TRUE(vane < cfg.vaneOpenPercent);  // closing, not opening
}

// BPR above target (drive too high) -> open vanes (clamps fully open).
void test_bpr_above_target_opens(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 20.0f;
    in.tipGaugePsi = 30.0f;  // bpr = 1.5, error = -0.5
    BoostState st = {0.0f, false, true};
    uint8_t vane = boostBprStep(in, cfg, st, 0.01f);
    // negative error -> integral clamps to 0, vane wants fully open but is held at
    // the controller open-authority cap.
    TEST_ASSERT_EQUAL_UINT8(cfg.vaneOpenCapPercent, vane);
}

// A bigger BPR deficit closes the vanes more.
void test_bigger_bpr_deficit_closes_more(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs small;
    small.boostGaugePsi = 20.0f;
    small.tipGaugePsi = 18.0f;  // bpr = 0.9, error = 0.1
    BoostState stSmall = {40.0f, false, true};  // wound integral, below the open cap
    uint8_t vaneSmall = boostBprStep(small, cfg, stSmall, 0.01f);

    BoostInputs big;
    big.boostGaugePsi = 20.0f;
    big.tipGaugePsi = 10.0f;   // bpr = 0.5, error = 0.5
    BoostState stBig = {40.0f, false, true};  // wound integral, below the open cap
    uint8_t vaneBig = boostBprStep(big, cfg, stBig, 0.01f);

    TEST_ASSERT_TRUE(vaneBig < vaneSmall);  // more deficit => more closure => lower %
}

// Handoff from spool to PI starts at the spool position, NOT a jump to fully open.
void test_bumpless_handoff_from_spool(void) {
    BoostConfig cfg = makeConfig();
    BoostState st = {0.0f, false, false};

    BoostInputs spool;
    spool.boostGaugePsi = 1.0f;  // spool region (< boostSpoolPsi)
    spool.tipGaugePsi = 20.0f;
    uint8_t vaneSpool = boostBprStep(spool, cfg, st, 0.01f);
    TEST_ASSERT_EQUAL_UINT8(25, vaneSpool);

    BoostInputs handoff;
    handoff.boostGaugePsi = 20.0f;  // now above boostPiPsi -> PI region
    handoff.tipGaugePsi = 20.0f;    // bpr = 1.0, error = 0
    uint8_t vaneHandoff = boostBprStep(handoff, cfg, st, 0.01f);
    // seeded integral = open(88) - spool(25) = 63; closure = 20*0 + 63 = 63
    // vane = 88 - 63 = 25  (continues from spool, no jump to 88)
    TEST_ASSERT_EQUAL_UINT8(25, vaneHandoff);
    TEST_ASSERT_TRUE(vaneHandoff < cfg.vaneOpenPercent);
}

// Regression (boost "stick"): at the spool->PI handoff, drive pressure leads
// boost so BPR is transiently huge. The PI proportional term used to fling the
// vane wide open (25 -> ~76), collapsing drive and stalling the spool. Below
// spoolProtectBoostPsi the vane must NOT open past the spool position.
void test_spool_protect_prevents_handoff_open_kick(void) {
    BoostConfig cfg = makeConfig();
    BoostState st = {0.0f, false, false};

    // Spool: vane held closed at 25 while drive races ahead of boost.
    BoostInputs spool;
    spool.boostGaugePsi = 1.0f;
    spool.tipGaugePsi = 12.0f;
    boostBprStep(spool, cfg, st, 0.01f);

    // Handoff: boost just crosses into PI (3.3 psi, below spoolProtectBoostPsi)
    // while drive still leads (tip 11.6 -> bpr ~3.5, far above target).
    BoostInputs handoff;
    handoff.boostGaugePsi = 3.3f;
    handoff.tipGaugePsi = 11.6f;
    uint8_t vane = boostBprStep(handoff, cfg, st, 0.01f);
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(cfg.spoolPercent, vane);  // no open-kick
}

// Above spoolProtectBoostPsi the clamp releases: normal PI may open past spool
// (e.g. sustained cruise at a lower BPR target needs the vane open past 25).
void test_spool_protect_releases_above_threshold(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 20.0f;      // well above spoolProtectBoostPsi
    in.tipGaugePsi = 30.0f;        // bpr = 1.5 > target -> wants to open
    BoostState st = {0.0f, false, true};
    uint8_t vane = boostBprStep(in, cfg, st, 0.01f);
    TEST_ASSERT_GREATER_THAN_UINT8(cfg.spoolPercent, vane);  // released: opened past spool
}

// Above spoolProtectBoostPsi the spool clamp releases, but if drive still leads
// boost the PI proportional term slams the vane fully open (88), dumping drive.
// The open-authority cap (vaneOpenCapPercent) must limit that slam.
void test_vane_open_cap_limits_slam(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 8.0f;   // above spoolProtectBoostPsi -> spool clamp released
    in.tipGaugePsi = 26.0f;    // bpr ~3.25 >> target -> wants to slam wide open
    BoostState st = {0.0f, false, true};
    uint8_t vane = boostBprStep(in, cfg, st, 0.01f);
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(cfg.vaneOpenCapPercent, vane);
}

// The PI demand must not slam fully closed: it is floored at the spool position
// (same spoolPercent variable), so even a limit cycle only swings spool<->cap.
void test_vane_floored_at_spool_position(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 20.0f;  // high boost, PI, above spool-protect
    in.tipGaugePsi = 20.0f;    // bpr = 1.0, error = +0.5 -> wants to close
    BoostState st = {88.0f, false, true};  // integral wound to full travel
    uint8_t vane = boostBprStep(in, cfg, st, 0.01f);
    // closure is huge; vane would drive to 0 but is floored at the spool position
    TEST_ASSERT_EQUAL_UINT8(cfg.spoolPercent, vane);
}

// Persistent large deficit: integral winds up to full travel, vanes fully closed.
void test_integral_antiwindup_clamps_closed(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 20.0f;
    in.tipGaugePsi = 0.0f;  // bpr = 0, error = 1.0 (persistent)
    BoostState st = {0.0f, false, true};
    uint8_t vane = 0;
    for (int i = 0; i < 1000; i++) {
        vane = boostBprStep(in, cfg, st, 0.01f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 88.0f, st.integralTerm);  // clamped at full travel
    TEST_ASSERT_EQUAL_UINT8(cfg.spoolPercent, vane);           // floored at spool, not 0
}

// Hysteresis: once in PI, a boost dip into the dead band (between the two
// thresholds) keeps the PI region instead of chattering back to spool.
void test_hysteresis_stays_pi_in_band(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 2.0f;  // between boostSpoolPsi(1.5) and boostPiPsi(3.0)
    in.tipGaugePsi = 2.0f;
    BoostState st = {0.0f, false, true};
    boostBprStep(in, cfg, st, 0.01f);
    TEST_ASSERT_TRUE(st.inPiRegion);  // held PI, no chatter to spool
}

// Hysteresis: below the low threshold, fall back to spool.
void test_hysteresis_drops_to_spool_below_low(void) {
    BoostConfig cfg = makeConfig();
    BoostInputs in;
    in.boostGaugePsi = 1.0f;  // < boostSpoolPsi
    in.tipGaugePsi = 1.0f;
    BoostState st = {0.0f, false, true};
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
    BoostState st = {0.0f, false, false};
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
    BoostState st = {0.0f, false, false};
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
    RUN_TEST(test_spool_protect_prevents_handoff_open_kick);
    RUN_TEST(test_spool_protect_releases_above_threshold);
    RUN_TEST(test_vane_open_cap_limits_slam);
    RUN_TEST(test_vane_floored_at_spool_position);
    RUN_TEST(test_integral_antiwindup_clamps_closed);
    RUN_TEST(test_hysteresis_stays_pi_in_band);
    RUN_TEST(test_hysteresis_drops_to_spool_below_low);
    RUN_TEST(test_hysteresis_stays_spool_in_band);
    RUN_TEST(test_hysteresis_enters_pi_above_high);
    return UNITY_END();
}
