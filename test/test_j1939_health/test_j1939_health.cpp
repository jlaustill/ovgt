#include <unity.h>
#include "sensors/j1939Health.h"

void setUp(void) {}
void tearDown(void) {}

void test_raw1_ok(void)  { TEST_ASSERT_EQUAL_INT(J1939_OK,  j1939RawStatus1Byte(0x64)); }
void test_raw1_na(void)  { TEST_ASSERT_EQUAL_INT(J1939_NA,  j1939RawStatus1Byte(0xFF)); }
void test_raw1_err(void) { TEST_ASSERT_EQUAL_INT(J1939_ERR, j1939RawStatus1Byte(0xFE)); }

void test_raw2_ok(void)          { TEST_ASSERT_EQUAL_INT(J1939_OK,  j1939RawStatus2Byte(0x2000)); }
void test_raw2_na_boundary(void) { TEST_ASSERT_EQUAL_INT(J1939_NA,  j1939RawStatus2Byte(0xFF00)); }
void test_raw2_na_full(void)     { TEST_ASSERT_EQUAL_INT(J1939_NA,  j1939RawStatus2Byte(0xFFFF)); }
void test_raw2_err_low(void)     { TEST_ASSERT_EQUAL_INT(J1939_ERR, j1939RawStatus2Byte(0xFE00)); }
void test_raw2_err_high(void)    { TEST_ASSERT_EQUAL_INT(J1939_ERR, j1939RawStatus2Byte(0xFEFF)); }

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_raw1_ok);
    RUN_TEST(test_raw1_na);
    RUN_TEST(test_raw1_err);
    RUN_TEST(test_raw2_ok);
    RUN_TEST(test_raw2_na_boundary);
    RUN_TEST(test_raw2_na_full);
    RUN_TEST(test_raw2_err_low);
    RUN_TEST(test_raw2_err_high);
    return UNITY_END();
}
