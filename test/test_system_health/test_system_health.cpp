#include <unity.h>
#include "systemHealthLogic.h"

void setUp(void) {}
void tearDown(void) {}

// One SRSR bit per known reset cause -> decoded code.
void test_decode_each_cause(void) {
    TEST_ASSERT_EQUAL_UINT8(1, SystemHealthLogic_decodeResetCause(1u << 0)); // por
    TEST_ASSERT_EQUAL_UINT8(2, SystemHealthLogic_decodeResetCause(1u << 1)); // lockup
    TEST_ASSERT_EQUAL_UINT8(3, SystemHealthLogic_decodeResetCause(1u << 3)); // pin
    TEST_ASSERT_EQUAL_UINT8(4, SystemHealthLogic_decodeResetCause(1u << 4)); // wdog
    TEST_ASSERT_EQUAL_UINT8(5, SystemHealthLogic_decodeResetCause(1u << 7)); // wdog3
    TEST_ASSERT_EQUAL_UINT8(6, SystemHealthLogic_decodeResetCause(1u << 8)); // tempsense
    TEST_ASSERT_EQUAL_UINT8(7, SystemHealthLogic_decodeResetCause(1u << 5)); // jtag
    TEST_ASSERT_EQUAL_UINT8(8, SystemHealthLogic_decodeResetCause(1u << 2)); // csu
    TEST_ASSERT_EQUAL_UINT8(0, SystemHealthLogic_decodeResetCause(0u));      // unknown
}

// Watchdog wins when both watchdog and POR bits are set (real resets set several).
void test_decode_precedence_wdog_over_por(void) {
    uint32_t srsr = (1u << 4) | (1u << 0);
    TEST_ASSERT_EQUAL_UINT8(4, SystemHealthLogic_decodeResetCause(srsr));
}

void test_loop_timing_max_and_avg(void) {
    SystemHealthLogic_loopTimingReset();
    SystemHealthLogic_loopTimingRecord(10000);
    SystemHealthLogic_loopTimingRecord(12000);
    SystemHealthLogic_loopTimingRecord(11000);
    TEST_ASSERT_EQUAL_UINT32(12000, SystemHealthLogic_loopTimingMax());
    TEST_ASSERT_EQUAL_UINT32(11000, SystemHealthLogic_loopTimingAvg()); // (10000+12000+11000)/3
}

void test_loop_timing_reset_clears(void) {
    SystemHealthLogic_loopTimingReset();
    SystemHealthLogic_loopTimingRecord(50000);
    SystemHealthLogic_loopTimingReset();
    TEST_ASSERT_EQUAL_UINT32(0, SystemHealthLogic_loopTimingMax());
    TEST_ASSERT_EQUAL_UINT32(0, SystemHealthLogic_loopTimingAvg());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_decode_each_cause);
    RUN_TEST(test_decode_precedence_wdog_over_por);
    RUN_TEST(test_loop_timing_max_and_avg);
    RUN_TEST(test_loop_timing_reset_clears);
    return UNITY_END();
}
