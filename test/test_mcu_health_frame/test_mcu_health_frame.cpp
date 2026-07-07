#include <unity.h>
#include "sensors/mcuHealthFrame.h"
#include <J1939McuHealth.h>

void setUp(void) {}
void tearDown(void) {}

// Maps fields and encodes to the expected 8 bytes (temp 60C, wdog, boot 7, uptime 1000).
void test_build_and_encode(void) {
    McuHealth h = buildMcuHealth(60.0f, 4 /*wdog*/, 7, 1000);
    uint8_t buf[8];
    J1939McuHealth_encode(&h, buf);
    TEST_ASSERT_EQUAL_UINT8(0xA0, buf[0]);   // 60C -> raw 0x29A0, LE
    TEST_ASSERT_EQUAL_UINT8(0x29, buf[1]);
    TEST_ASSERT_EQUAL_UINT8(4,    buf[2]);
    TEST_ASSERT_EQUAL_UINT8(0x07, buf[3]);
    TEST_ASSERT_EQUAL_UINT8(0x00, buf[4]);
    TEST_ASSERT_EQUAL_UINT8(0xE8, buf[5]);   // 1000 -> 0x03E8, LE
    TEST_ASSERT_EQUAL_UINT8(0x03, buf[6]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, buf[7]);
}

// u32 sources cap at 0xFFFE so a real value never masquerades as the 0xFFFF N/A sentinel.
void test_caps_below_sentinel(void) {
    McuHealth h = buildMcuHealth(20.0f, 0, 0xFFFFFFFFu, 0x10000u);
    TEST_ASSERT_EQUAL_UINT16(0xFFFE, h.bootCount);
    TEST_ASSERT_EQUAL_UINT16(0xFFFE, h.uptimeMinutes);
    TEST_ASSERT_TRUE(h.bootCountValid);
    TEST_ASSERT_TRUE(h.uptimeValid);
    TEST_ASSERT_TRUE(h.temperatureValid);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_build_and_encode);
    RUN_TEST(test_caps_below_sentinel);
    return UNITY_END();
}
