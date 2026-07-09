#include <unity.h>
#include "sensors/cotThermistor.h"

void setUp(void) {}
void tearDown(void) {}

// Datasheet points (pull-up 1.0k, Vcc 5): R -> divider V -> °C, within 0.5 °C.
void test_datasheet_points(void) {
    float t;
    TEST_ASSERT_TRUE(cotThermistorReadC(4.3714f, &t));   // 6954 Ω, 200 °F
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 93.333f, t);
    TEST_ASSERT_TRUE(cotThermistorReadC(2.9236f, &t));   // 1408 Ω, 305 °F
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 151.667f, t);
    TEST_ASSERT_TRUE(cotThermistorReadC(0.8609f, &t));   // 208 Ω, 485 °F
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 251.667f, t);
}

// Railed samples (open/short) are rejected and leave the output untouched.
void test_rail_reject_holds_last(void) {
    float t = -999.0f;
    TEST_ASSERT_FALSE(cotThermistorReadC(4.99f, &t));    // open (near 5 V)
    TEST_ASSERT_EQUAL_FLOAT(-999.0f, t);
    TEST_ASSERT_FALSE(cotThermistorReadC(0.01f, &t));    // short (near 0 V)
    TEST_ASSERT_EQUAL_FLOAT(-999.0f, t);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_datasheet_points);
    RUN_TEST(test_rail_reject_holds_last);
    return UNITY_END();
}
