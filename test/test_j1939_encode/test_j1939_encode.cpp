#include <unity.h>
#include "sensors/j1939Encode.h"

void setUp(void) {}
void tearDown(void) {}

// 0.03125 deg C/bit, offset -273  =>  raw = (C + 273) * 32
void test_encode_temperature(void) {
    TEST_ASSERT_EQUAL_UINT16(8736, j1939EncodeTemperatureRaw(0.0f));    // 273*32
    TEST_ASSERT_EQUAL_UINT16(9536, j1939EncodeTemperatureRaw(25.0f));   // 298*32
    TEST_ASSERT_EQUAL_UINT16(24736, j1939EncodeTemperatureRaw(500.0f)); // 773*32
}

// SPN 1127 boost: 0.125 kPa/bit, offset 0. input gauge hPa => raw = hPa * 0.8
void test_encode_boost(void) {
    TEST_ASSERT_EQUAL_UINT16(0, j1939EncodeBoostRaw(0.0f));
    TEST_ASSERT_EQUAL_UINT16(800, j1939EncodeBoostRaw(1000.0f));   // ~14.5 psi
    TEST_ASSERT_EQUAL_UINT16(1600, j1939EncodeBoostRaw(2000.0f));  // ~29 psi
}

// SPN 1176/1209: 1/128 kPa/bit, offset -250 kPa. raw = (hPa*0.1 + 250) * 128
void test_encode_turbo_pressure(void) {
    TEST_ASSERT_EQUAL_UINT16(32000, j1939EncodeTurboPressureRaw(0.0f));   // (0+250)*128
    TEST_ASSERT_EQUAL_UINT16(43520, j1939EncodeTurboPressureRaw(900.0f)); // (90+250)*128
}

// clamp to the max valid 2-byte value; never spill into the 0xFB00+ reserved range
void test_encode_clamps_high(void) {
    TEST_ASSERT_EQUAL_UINT16(0xFAFF, j1939EncodeTemperatureRaw(5000.0f));
}

// negative / below-range clamps to 0
void test_encode_clamps_low(void) {
    TEST_ASSERT_EQUAL_UINT16(0, j1939EncodeBoostRaw(-100.0f));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_encode_temperature);
    RUN_TEST(test_encode_boost);
    RUN_TEST(test_encode_turbo_pressure);
    RUN_TEST(test_encode_clamps_high);
    RUN_TEST(test_encode_clamps_low);
    return UNITY_END();
}
