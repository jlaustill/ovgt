#include <unity.h>
#include "sensors/compressorEfficiency.h"

void setUp(void) {}
void tearDown(void) {}

// Pr=2.0, 20C in, 120C out:
//   ideal rise = 293.15 * (2^0.286 - 1) = 293.15 * 0.21926 = 64.28 K
//   actual rise = 100 K  ->  e = 0.6428
void test_known_efficiency(void) {
    float e = compressorEfficiency(2.0f, 20.0f, 120.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.643f, e);
}

// At/below unity pressure ratio (no compression) -> undefined (negative).
void test_undefined_at_or_below_unity_br(void) {
    TEST_ASSERT_TRUE(compressorEfficiency(1.00f, 20.0f, 24.0f) < 0.0f);
    TEST_ASSERT_TRUE(compressorEfficiency(0.98f, 20.0f, 24.0f) < 0.0f);
}

// Just above unity BR now computes (no blanking). Pr=1.02, 20C in, 25C out:
//   ideal rise = 293.15*(1.02^0.286 - 1) = 1.665 K; actual rise = 5 K -> 0.333
void test_computes_just_above_unity_br(void) {
    float e = compressorEfficiency(1.02f, 20.0f, 25.0f);
    TEST_ASSERT_TRUE(e > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.333f, e);
}

// Real compression but ~no temp rise (To <= Ti) -> 0/0 guard -> undefined.
void test_undefined_when_no_temp_rise(void) {
    float e = compressorEfficiency(2.0f, 50.0f, 50.0f);
    TEST_ASSERT_TRUE(e < 0.0f);
}

// Higher pressure ratio, plausible temps -> sane efficiency in (0,1).
void test_efficiency_in_range(void) {
    float e = compressorEfficiency(2.5f, 30.0f, 140.0f);
    TEST_ASSERT_TRUE(e > 0.0f);
    TEST_ASSERT_TRUE(e < 1.0f);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_known_efficiency);
    RUN_TEST(test_undefined_at_or_below_unity_br);
    RUN_TEST(test_computes_just_above_unity_br);
    RUN_TEST(test_undefined_when_no_temp_rise);
    RUN_TEST(test_efficiency_in_range);
    return UNITY_END();
}
