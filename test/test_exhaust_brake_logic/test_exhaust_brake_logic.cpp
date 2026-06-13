#include <unity.h>
#include "control/exhaustBrakeLogic.h"

void setUp(void) {}
void tearDown(void) {}

static BrakeConfig makeConfig(void) {
    BrakeConfig cfg;
    cfg.targetPsi = 60.0f;
    cfg.ceilingPsi = 65.0f;
    cfg.minVanePercent = 18;
    cfg.maxVanePercent = 68;
    cfg.staleTimeoutMs = 250;
    cfg.kp = 0.5f;
    cfg.ki = 0.3f;
    return cfg;
}

// Baseline: brake fully engaged, fresh CAN, backpressure at zero.
static BrakeInputs makeEngagedInputs(void) {
    BrakeInputs in;
    in.torqueConverterLockupStatus = 1;
    in.acceleratorPedalPercent = 0;
    in.engineLoadPercent = 0;
    in.tipGaugePsi = 0.0f;
    in.nowMs = 1000;
    in.lastEec2RxMs = 1000;
    in.lastEtc1RxMs = 1000;
    in.manualMode = false;
    return in;
}

void test_manual_mode_disengages(void) {
    BrakeInputs in = makeEngagedInputs();
    in.manualMode = true;
    BrakeConfig cfg = makeConfig();
    BrakeState st = {5.0f};
    BrakeOutput out = exhaustBrakeStep(in, cfg, st, 0.01f);
    TEST_ASSERT_FALSE(out.active);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, st.integralTerm);  // integral reset
}

void test_stale_eec2_disengages(void) {
    BrakeInputs in = makeEngagedInputs();
    in.lastEec2RxMs = 700;  // 1000 - 700 = 300 > 250
    BrakeConfig cfg = makeConfig();
    BrakeState st = {0.0f};
    BrakeOutput out = exhaustBrakeStep(in, cfg, st, 0.01f);
    TEST_ASSERT_FALSE(out.active);
}

void test_stale_etc1_disengages(void) {
    BrakeInputs in = makeEngagedInputs();
    in.lastEtc1RxMs = 700;
    BrakeConfig cfg = makeConfig();
    BrakeState st = {0.0f};
    BrakeOutput out = exhaustBrakeStep(in, cfg, st, 0.01f);
    TEST_ASSERT_FALSE(out.active);
}

void test_throttle_releases(void) {
    BrakeInputs in = makeEngagedInputs();
    in.acceleratorPedalPercent = 10;
    BrakeConfig cfg = makeConfig();
    BrakeState st = {5.0f};
    BrakeOutput out = exhaustBrakeStep(in, cfg, st, 0.01f);
    TEST_ASSERT_FALSE(out.active);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, st.integralTerm);
}

void test_load_releases(void) {
    BrakeInputs in = makeEngagedInputs();
    in.engineLoadPercent = 5;
    BrakeConfig cfg = makeConfig();
    BrakeState st = {0.0f};
    BrakeOutput out = exhaustBrakeStep(in, cfg, st, 0.01f);
    TEST_ASSERT_FALSE(out.active);
}

void test_not_locked_inactive(void) {
    BrakeInputs in = makeEngagedInputs();
    in.torqueConverterLockupStatus = 0;
    BrakeConfig cfg = makeConfig();
    BrakeState st = {0.0f};
    BrakeOutput out = exhaustBrakeStep(in, cfg, st, 0.01f);
    TEST_ASSERT_FALSE(out.active);
}

void test_lockup_error_state_inactive(void) {
    BrakeInputs in = makeEngagedInputs();
    in.torqueConverterLockupStatus = 2;  // error state, not engaged
    BrakeConfig cfg = makeConfig();
    BrakeState st = {0.0f};
    BrakeOutput out = exhaustBrakeStep(in, cfg, st, 0.01f);
    TEST_ASSERT_FALSE(out.active);
}

void test_engaged_drives_vanes(void) {
    BrakeInputs in = makeEngagedInputs();  // tip = 0, error = 60
    BrakeConfig cfg = makeConfig();
    BrakeState st = {0.0f};
    BrakeOutput out = exhaustBrakeStep(in, cfg, st, 0.01f);
    TEST_ASSERT_TRUE(out.active);
    TEST_ASSERT_FALSE(out.fault);
    // vane = kp*error + integral = 0.5*60 + 0.3*60*0.01 = 30.18 -> 30
    TEST_ASSERT_EQUAL_UINT8(30, out.vanePercent);
}

void test_ceiling_opens_vanes(void) {
    BrakeInputs in = makeEngagedInputs();
    in.tipGaugePsi = 70.0f;  // above 65 ceiling
    BrakeConfig cfg = makeConfig();
    BrakeState st = {10.0f};
    BrakeOutput out = exhaustBrakeStep(in, cfg, st, 0.01f);
    TEST_ASSERT_TRUE(out.active);
    TEST_ASSERT_TRUE(out.fault);
    TEST_ASSERT_EQUAL_UINT8(18, out.vanePercent);  // forced open to minVane
}

void test_integral_antiwindup_clamps(void) {
    BrakeInputs in = makeEngagedInputs();  // persistent large error
    BrakeConfig cfg = makeConfig();
    BrakeState st = {0.0f};
    BrakeOutput out;
    for (int i = 0; i < 1000; i++) {
        out = exhaustBrakeStep(in, cfg, st, 0.01f);
    }
    TEST_ASSERT_TRUE(out.active);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 68.0f, st.integralTerm);  // clamped at maxVane
    TEST_ASSERT_EQUAL_UINT8(68, out.vanePercent);              // output clamped too
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_manual_mode_disengages);
    RUN_TEST(test_stale_eec2_disengages);
    RUN_TEST(test_stale_etc1_disengages);
    RUN_TEST(test_throttle_releases);
    RUN_TEST(test_load_releases);
    RUN_TEST(test_not_locked_inactive);
    RUN_TEST(test_lockup_error_state_inactive);
    RUN_TEST(test_engaged_drives_vanes);
    RUN_TEST(test_ceiling_opens_vanes);
    RUN_TEST(test_integral_antiwindup_clamps);
    return UNITY_END();
}
