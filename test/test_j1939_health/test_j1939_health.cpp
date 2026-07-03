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

void test_domain_offline_never_seen(void) {
    TEST_ASSERT_FALSE(j1939DomainOnline(0, 0, 5000, 1000));
}
void test_domain_online_one_gate(void) {
    TEST_ASSERT_TRUE(j1939DomainOnline(4500, 0, 5000, 1000));  // gate A 500ms ago
}
void test_domain_offline_stale(void) {
    TEST_ASSERT_FALSE(j1939DomainOnline(3000, 0, 5000, 1000)); // 2000ms ago > 1000
}
void test_signal_waiting_when_domain_offline(void) {
    TEST_ASSERT_EQUAL_INT(J1939_STATUS_WAITING,
        j1939SignalStatus(false, 4900, 5000, 1000, J1939_OK));
}
void test_signal_absent_when_online_but_never_seen(void) {
    TEST_ASSERT_EQUAL_INT(J1939_STATUS_ABSENT,
        j1939SignalStatus(true, 0, 5000, 1000, J1939_OK));
}
void test_signal_absent_when_online_but_stale(void) {
    TEST_ASSERT_EQUAL_INT(J1939_STATUS_ABSENT,
        j1939SignalStatus(true, 3000, 5000, 1000, J1939_OK));
}
void test_signal_ok_when_fresh(void) {
    TEST_ASSERT_EQUAL_INT(J1939_STATUS_OK,
        j1939SignalStatus(true, 4800, 5000, 1000, J1939_OK));
}
void test_signal_na_maps_through(void) {
    TEST_ASSERT_EQUAL_INT(J1939_STATUS_NA,
        j1939SignalStatus(true, 4800, 5000, 1000, J1939_NA));
}
void test_signal_err_maps_through(void) {
    TEST_ASSERT_EQUAL_INT(J1939_STATUS_ERR,
        j1939SignalStatus(true, 4800, 5000, 1000, J1939_ERR));
}
void test_status_names(void) {
    TEST_ASSERT_EQUAL_STRING("ok",      j1939StatusName(J1939_STATUS_OK));
    TEST_ASSERT_EQUAL_STRING("na",      j1939StatusName(J1939_STATUS_NA));
    TEST_ASSERT_EQUAL_STRING("err",     j1939StatusName(J1939_STATUS_ERR));
    TEST_ASSERT_EQUAL_STRING("absent",  j1939StatusName(J1939_STATUS_ABSENT));
    TEST_ASSERT_EQUAL_STRING("waiting", j1939StatusName(J1939_STATUS_WAITING));
}

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
    RUN_TEST(test_domain_offline_never_seen);
    RUN_TEST(test_domain_online_one_gate);
    RUN_TEST(test_domain_offline_stale);
    RUN_TEST(test_signal_waiting_when_domain_offline);
    RUN_TEST(test_signal_absent_when_online_but_never_seen);
    RUN_TEST(test_signal_absent_when_online_but_stale);
    RUN_TEST(test_signal_ok_when_fresh);
    RUN_TEST(test_signal_na_maps_through);
    RUN_TEST(test_signal_err_maps_through);
    RUN_TEST(test_status_names);
    return UNITY_END();
}
