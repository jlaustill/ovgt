#include <unity.h>
#include "json.h"

void setUp(void) {}
void tearDown(void) {}

// Render the builder's buffer as a C string for comparison.
static const char *render(void) {
    static char buf[700];
    uint32_t n = Json_len();
    for (uint32_t i = 0; i < n; i++) {
        buf[i] = (char)Json_at(i);
    }
    buf[n] = '\0';
    return buf;
}

// Every field type, separators, negatives, floats, strings.
void test_field_api(void) {
    Json_begin();
    Json_addUint("a", 12);
    Json_addBool("b", true);
    Json_addInt("c", -7);
    Json_addFloat2("d", 3.14f);
    Json_addStr("m", "auto");
    Json_end();
    TEST_ASSERT_EQUAL_STRING("{\"a\":12,\"b\":true,\"c\":-7,\"d\":3.14,\"m\":\"auto\"}", render());
}

// Zero value and fractional zero-padding.
void test_zero_and_pad(void) {
    Json_begin();
    Json_addUint("z", 0);
    Json_addFloat2("p", 0.07f);
    Json_end();
    TEST_ASSERT_EQUAL_STRING("{\"z\":0,\"p\":0.07}", render());
}

// Worst-case telemetry object (21 fields, max widths) must fit string<640>.
void test_capacity_no_overflow(void) {
    Json_begin();
    Json_addStr("type", "t");
    Json_addUint("t_ms", 4294967295u);
    Json_addStr("mode", "manual");
    Json_addUint("cop_hpa", 65535);
    Json_addUint("cip_hpa", 65535);
    Json_addFloat2("boost_psi", 9999.99f);
    Json_addFloat2("br", 9999.99f);
    Json_addFloat2("tip_psi", 9999.99f);
    Json_addFloat2("bpr", 9999.99f);
    Json_addFloat2("bpr_target", 9999.99f);
    Json_addFloat2("cit_c", -999.99f);
    Json_addFloat2("cot_c", 9999.99f);
    Json_addInt("tit_c", -32768);
    Json_addFloat2("ce_pct", 100.0f);
    Json_addBool("ce_settled", false);
    Json_addUint("dem_pct", 100);
    Json_addUint("pos_pct", 100);
    Json_addBool("brake", false);
    Json_addFloat2("cot_slope_c_s", -999.99f);
    Json_addFloat2("boost_slope_psi_s", -999.99f);
    Json_addFloat2("settle_timer_s", 9999.99f);
    Json_addStr("reset_cause", "tempsense");
    Json_addUint("boot_count", 4294967295u);
    Json_addBool("crash", true);
    Json_addBool("pg", true);
    Json_addInt("vin_mv", 60000);
    Json_addUint("loop_us_max", 4294967295u);
    Json_addUint("loop_us_avg", 4294967295u);
    Json_addUint("setup_ms", 4294967295u);
    Json_end();
    TEST_ASSERT_FALSE(Json_overflowed());
    TEST_ASSERT_TRUE(Json_len() < 640u);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_field_api);
    RUN_TEST(test_zero_and_pad);
    RUN_TEST(test_capacity_no_overflow);
    return UNITY_END();
}
