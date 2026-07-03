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

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_decode_each_cause);
    RUN_TEST(test_decode_precedence_wdog_over_por);
    return UNITY_END();
}
