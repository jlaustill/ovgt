#include <unity.h>
#include "sensors/j1939Decode.h"

void setUp(void) {}
void tearDown(void) {}

void test_pgn_eec2_pdu2(void) {
    // priority 6, PF 0xF0, PS 0x03, SA 0x00  -> PGN 61443
    uint32_t id = ((uint32_t)6 << 26) | ((uint32_t)0xF0 << 16) | ((uint32_t)0x03 << 8) | 0x00;
    TEST_ASSERT_EQUAL_UINT32(61443u, pgnFromCanId(id));
}

void test_pgn_etc1_pdu2(void) {
    // PF 0xF0, PS 0x02 -> PGN 61442
    uint32_t id = ((uint32_t)6 << 26) | ((uint32_t)0xF0 << 16) | ((uint32_t)0x02 << 8) | 0x21;
    TEST_ASSERT_EQUAL_UINT32(61442u, pgnFromCanId(id));
}

void test_pgn_pdu1_excludes_ps(void) {
    // PF 0xEF (239 < 240) -> PS ignored, PGN = 0xEF00
    uint32_t id = ((uint32_t)6 << 26) | ((uint32_t)0xEF << 16) | ((uint32_t)0x05 << 8) | 0x21;
    TEST_ASSERT_EQUAL_UINT32((uint32_t)0xEF00, pgnFromCanId(id));
}

void test_lockup_engaged(void) {
    uint8_t buf[8] = {0x04, 0, 0, 0, 0, 0, 0, 0};  // bits 3-4 = 0b01
    TEST_ASSERT_EQUAL_UINT8(1, decodeTorqueConverterLockup(buf));
}

void test_lockup_not_engaged(void) {
    uint8_t buf[8] = {0x00, 0, 0, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_UINT8(0, decodeTorqueConverterLockup(buf));
}

void test_lockup_error_state(void) {
    uint8_t buf[8] = {0x08, 0, 0, 0, 0, 0, 0, 0};  // bits 3-4 = 0b10
    TEST_ASSERT_EQUAL_UINT8(2, decodeTorqueConverterLockup(buf));
}

void test_accelerator_zero(void) {
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_UINT8(0, decodeAcceleratorPedalPercent(buf));
}

void test_accelerator_full(void) {
    uint8_t buf[8] = {0, 250, 0, 0, 0, 0, 0, 0};  // 250 * 0.4 = 100
    TEST_ASSERT_EQUAL_UINT8(100, decodeAcceleratorPedalPercent(buf));
}

void test_accelerator_half(void) {
    uint8_t buf[8] = {0, 125, 0, 0, 0, 0, 0, 0};  // 125 * 0.4 = 50
    TEST_ASSERT_EQUAL_UINT8(50, decodeAcceleratorPedalPercent(buf));
}

void test_engine_load(void) {
    uint8_t buf[8] = {0, 0, 100, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_UINT8(100, decodeEngineLoadPercent(buf));
}

void test_engine_rpm(void) {
    // 1500 rpm / 0.125 = 12000 = 0x2EE0 -> LE byte3=0xE0, byte4=0x2E
    uint8_t buf[8] = {0, 0, 0, 0xE0, 0x2E, 0, 0, 0};
    TEST_ASSERT_EQUAL_UINT16(1500, decodeEngineRpm(buf));
}

void test_driver_demand_torque(void) {
    uint8_t buf[8] = {0, 150, 0, 0, 0, 0, 0, 0};  // 150 - 125 = 25
    TEST_ASSERT_EQUAL_INT8(25, decodeDriverDemandTorquePct(buf));
}

void test_actual_torque_negative(void) {
    uint8_t buf[8] = {0, 0, 100, 0, 0, 0, 0, 0};  // 100 - 125 = -25 (motoring)
    TEST_ASSERT_EQUAL_INT8(-25, decodeActualTorquePct(buf));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_pgn_eec2_pdu2);
    RUN_TEST(test_pgn_etc1_pdu2);
    RUN_TEST(test_pgn_pdu1_excludes_ps);
    RUN_TEST(test_lockup_engaged);
    RUN_TEST(test_lockup_not_engaged);
    RUN_TEST(test_lockup_error_state);
    RUN_TEST(test_accelerator_zero);
    RUN_TEST(test_accelerator_full);
    RUN_TEST(test_accelerator_half);
    RUN_TEST(test_engine_load);
    RUN_TEST(test_engine_rpm);
    RUN_TEST(test_driver_demand_torque);
    RUN_TEST(test_actual_torque_negative);
    return UNITY_END();
}
