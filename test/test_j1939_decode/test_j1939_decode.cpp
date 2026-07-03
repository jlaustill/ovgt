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

void test_intake_air_temp(void) {
    uint8_t buf[8] = {0, 0, 80, 0, 0, 0, 0, 0};  // 80 - 40 = 40 C
    TEST_ASSERT_EQUAL_INT16(40, decodeIntakeAirTempC(buf));
}

void test_boost_kpa(void) {
    uint8_t buf[8] = {0, 100, 0, 0, 0, 0, 0, 0};  // 100 * 2 = 200 kPa
    TEST_ASSERT_EQUAL_UINT16(200, decodeBoostKpa(buf));
}

void test_preturbo_kpa(void) {
    uint8_t buf[8] = {200, 0, 0, 0, 0, 0, 0, 0};  // 200 * 0.5 = 100 kPa
    TEST_ASSERT_EQUAL_UINT16(100, decodePreTurboKpa(buf));
}

void test_coolant_temp(void) {
    uint8_t buf[8] = {130, 0, 0, 0, 0, 0, 0, 0};  // 130 - 40 = 90 C
    TEST_ASSERT_EQUAL_INT16(90, decodeCoolantTempC(buf));
}

void test_oil_temp(void) {
    // 100 C -> (100+273)/0.03125 = 11936 = 0x2EA0 -> LE byte2=0xA0, byte3=0x2E
    uint8_t buf[8] = {0, 0, 0xA0, 0x2E, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_INT16(100, decodeOilTempC(buf));
}

void test_oil_pressure(void) {
    uint8_t buf[8] = {0, 0, 0, 75, 0, 0, 0, 0};  // 75 * 4 = 300 kPa
    TEST_ASSERT_EQUAL_UINT16(300, decodeOilPressureKpa(buf));
}

void test_system_voltage(void) {
    // 13.8 V / 0.05 = 276 = 0x0114 -> LE byte4=0x14, byte5=0x01
    uint8_t buf[8] = {0, 0, 0, 0, 0x14, 0x01, 0, 0};
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 13.8f, decodeSystemVoltage(buf));
}

void test_output_shaft_rpm(void) {
    // 2000 rpm / 0.125 = 16000 = 0x3E80 -> LE byte1=0x80, byte2=0x3E
    uint8_t buf[8] = {0, 0x80, 0x3E, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_UINT16(2000, decodeOutputShaftRpm(buf));
}

void test_input_shaft_rpm(void) {
    // 2400 rpm / 0.125 = 19200 = 0x4B00 -> LE byte5=0x00, byte6=0x4B (SAE pos 6-7)
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0x00, 0x4B, 0};
    TEST_ASSERT_EQUAL_UINT16(2400, decodeInputShaftRpm(buf));
}

void test_clutch_slip(void) {
    uint8_t buf[8] = {0, 0, 0, 25, 0, 0, 0, 0};  // 25 * 0.4 = 10 %
    TEST_ASSERT_EQUAL_UINT8(10, decodeClutchSlipPct(buf));
}

void test_selected_gear_raw(void) {
    uint8_t buf[8] = {0x7E, 0, 0, 0, 0, 0, 0, 0};  // 0x7E = 1st (Allison)
    TEST_ASSERT_EQUAL_UINT8(0x7E, decodeSelectedGear(buf));
}

void test_current_gear_raw(void) {
    uint8_t buf[8] = {0, 0, 0, 0x7D, 0, 0, 0, 0};  // byte3 = 0x7D = Neutral
    TEST_ASSERT_EQUAL_UINT8(0x7D, decodeCurrentGear(buf));
}

void test_gear_ratio_milli(void) {
    // 1.000 ratio -> 1000 = 0x03E8 -> LE byte1=0xE8, byte2=0x03
    uint8_t buf[8] = {0, 0xE8, 0x03, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_UINT16(1000, decodeGearRatioMilli(buf));
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
    RUN_TEST(test_intake_air_temp);
    RUN_TEST(test_boost_kpa);
    RUN_TEST(test_preturbo_kpa);
    RUN_TEST(test_coolant_temp);
    RUN_TEST(test_oil_temp);
    RUN_TEST(test_oil_pressure);
    RUN_TEST(test_system_voltage);
    RUN_TEST(test_output_shaft_rpm);
    RUN_TEST(test_input_shaft_rpm);
    RUN_TEST(test_clutch_slip);
    RUN_TEST(test_selected_gear_raw);
    RUN_TEST(test_current_gear_raw);
    RUN_TEST(test_gear_ratio_milli);
    return UNITY_END();
}
