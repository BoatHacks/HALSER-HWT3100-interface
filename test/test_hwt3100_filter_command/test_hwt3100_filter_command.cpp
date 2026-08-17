#include <cstring>

#include <unity.h>

#include "hwt3100_filter_command.h"

using halser::FormatFilterCommand;

void setUp(void) {}
void tearDown(void) {}

static void test_typical_value(void) {
  char buf[24];
  uint16_t clamped = FormatFilterCommand(500, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT16(500, clamped);
  TEST_ASSERT_EQUAL_STRING("AT+FILT=500\r\n", buf);
}

static void test_zero_is_off_not_clamped(void) {
  // 0 is a legitimate "no filter" command per the manual, not
  // out-of-range input to be adjusted.
  char buf[24];
  uint16_t clamped = FormatFilterCommand(0, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT16(0, clamped);
  TEST_ASSERT_EQUAL_STRING("AT+FILT=0\r\n", buf);
}

static void test_negative_clamps_to_zero(void) {
  char buf[24];
  uint16_t clamped = FormatFilterCommand(-5, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT16(0, clamped);
  TEST_ASSERT_EQUAL_STRING("AT+FILT=0\r\n", buf);
}

static void test_over_max_clamps_to_999(void) {
  // There is no AT+FILT=1000 in the documented command set (only
  // AT+FILT=0 and AT+FILT=<1-999>) - anything above 999 clamps down to
  // the actual maximum, 999.
  char buf[24];
  uint16_t clamped = FormatFilterCommand(5000, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT16(999, clamped);
  TEST_ASSERT_EQUAL_STRING("AT+FILT=999\r\n", buf);
}

static void test_one_thousand_clamps_to_999(void) {
  char buf[24];
  uint16_t clamped = FormatFilterCommand(1000, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT16(999, clamped);
  TEST_ASSERT_EQUAL_STRING("AT+FILT=999\r\n", buf);
}

static void test_boundary_one(void) {
  char buf[24];
  uint16_t clamped = FormatFilterCommand(1, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT16(1, clamped);
  TEST_ASSERT_EQUAL_STRING("AT+FILT=1\r\n", buf);
}

static void test_boundary_999(void) {
  char buf[24];
  uint16_t clamped = FormatFilterCommand(999, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT16(999, clamped);
  TEST_ASSERT_EQUAL_STRING("AT+FILT=999\r\n", buf);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_typical_value);
  RUN_TEST(test_zero_is_off_not_clamped);
  RUN_TEST(test_negative_clamps_to_zero);
  RUN_TEST(test_over_max_clamps_to_999);
  RUN_TEST(test_one_thousand_clamps_to_999);
  RUN_TEST(test_boundary_one);
  RUN_TEST(test_boundary_999);
  return UNITY_END();
}
