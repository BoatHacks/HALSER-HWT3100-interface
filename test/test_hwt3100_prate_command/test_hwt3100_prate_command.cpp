#include <cstring>

#include <unity.h>

#include "hwt3100_prate_command.h"

using halser::FormatPrateCommand;
using halser::ParsePrateReply;

void setUp(void) {}
void tearDown(void) {}

// --- FormatPrateCommand ---

static void test_typical_value(void) {
  char buf[24];
  uint16_t clamped = FormatPrateCommand(100, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT16(100, clamped);
  TEST_ASSERT_EQUAL_STRING("AT+PRATE=100\r\n", buf);
}

static void test_zero_is_single_return_not_clamped(void) {
  char buf[24];
  uint16_t clamped = FormatPrateCommand(0, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT16(0, clamped);
  TEST_ASSERT_EQUAL_STRING("AT+PRATE=0\r\n", buf);
}

static void test_negative_clamps_to_zero(void) {
  char buf[24];
  uint16_t clamped = FormatPrateCommand(-5, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT16(0, clamped);
  TEST_ASSERT_EQUAL_STRING("AT+PRATE=0\r\n", buf);
}

static void test_small_positive_clamps_up_to_ten(void) {
  // 1-9 is not a valid AT+PRATE value (0, or 10-10000 only). Rounding
  // up to the minimum valid periodic interval, not down to "stop
  // streaming," is the less surprising behavior.
  char buf[24];
  uint16_t clamped = FormatPrateCommand(5, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT16(10, clamped);
  TEST_ASSERT_EQUAL_STRING("AT+PRATE=10\r\n", buf);
}

static void test_over_max_clamps_to_ten_thousand(void) {
  char buf[24];
  uint16_t clamped = FormatPrateCommand(50000, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT16(10000, clamped);
  TEST_ASSERT_EQUAL_STRING("AT+PRATE=10000\r\n", buf);
}

static void test_boundary_ten(void) {
  char buf[24];
  uint16_t clamped = FormatPrateCommand(10, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT16(10, clamped);
  TEST_ASSERT_EQUAL_STRING("AT+PRATE=10\r\n", buf);
}

static void test_boundary_ten_thousand(void) {
  char buf[24];
  uint16_t clamped = FormatPrateCommand(10000, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT16(10000, clamped);
  TEST_ASSERT_EQUAL_STRING("AT+PRATE=10000\r\n", buf);
}

// --- ParsePrateReply ---

static void test_parses_well_formed_reply(void) {
  int value = -999;
  TEST_ASSERT_TRUE(ParsePrateReply("+PRATE=100", &value));
  TEST_ASSERT_EQUAL_INT(100, value);
}

static void test_parses_zero_reply(void) {
  int value = -999;
  TEST_ASSERT_TRUE(ParsePrateReply("+PRATE=0", &value));
  TEST_ASSERT_EQUAL_INT(0, value);
}

static void test_parses_reply_with_trailing_crlf(void) {
  int value = -999;
  TEST_ASSERT_TRUE(ParsePrateReply("+PRATE=250\r\n", &value));
  TEST_ASSERT_EQUAL_INT(250, value);
}

static void test_rejects_ordinary_data_line(void) {
  // The overwhelming majority of lines on this stream are heading data,
  // not command replies - these must not be misparsed as a PRATE reply.
  int value = -999;
  TEST_ASSERT_FALSE(ParsePrateReply("Magx=1,y=2,z=3,w=4.5", &value));
  TEST_ASSERT_EQUAL_INT(-999, value);
}

static void test_rejects_malformed_reply(void) {
  int value = -999;
  TEST_ASSERT_FALSE(ParsePrateReply("+PRATE=", &value));
  TEST_ASSERT_EQUAL_INT(-999, value);
}

static void test_rejects_null_arguments(void) {
  int value = 0;
  TEST_ASSERT_FALSE(ParsePrateReply(nullptr, &value));
  TEST_ASSERT_FALSE(ParsePrateReply("+PRATE=100", nullptr));
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_typical_value);
  RUN_TEST(test_zero_is_single_return_not_clamped);
  RUN_TEST(test_negative_clamps_to_zero);
  RUN_TEST(test_small_positive_clamps_up_to_ten);
  RUN_TEST(test_over_max_clamps_to_ten_thousand);
  RUN_TEST(test_boundary_ten);
  RUN_TEST(test_boundary_ten_thousand);
  RUN_TEST(test_parses_well_formed_reply);
  RUN_TEST(test_parses_zero_reply);
  RUN_TEST(test_parses_reply_with_trailing_crlf);
  RUN_TEST(test_rejects_ordinary_data_line);
  RUN_TEST(test_rejects_malformed_reply);
  RUN_TEST(test_rejects_null_arguments);
  return UNITY_END();
}
