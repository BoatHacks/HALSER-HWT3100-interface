#include <unity.h>

#include "hwt3100_parser.h"
#include "hwt3100_types.h"

void setUp(void) {}
void tearDown(void) {}

static void test_well_formed_positive_values(void) {
  HeadingReading r;
  TEST_ASSERT_TRUE(ParseHWT3100Line("Magx=1234,y=567,z=89,w=102.9", &r));
  TEST_ASSERT_EQUAL_INT32(1234, r.mag_x);
  TEST_ASSERT_EQUAL_INT32(567, r.mag_y);
  TEST_ASSERT_EQUAL_INT32(89, r.mag_z);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 102.9f, r.heading);
}

static void test_negative_mag_values(void) {
  HeadingReading r;
  TEST_ASSERT_TRUE(ParseHWT3100Line("Magx=-1234,y=-567,z=-89,w=15.0", &r));
  TEST_ASSERT_EQUAL_INT32(-1234, r.mag_x);
  TEST_ASSERT_EQUAL_INT32(-567, r.mag_y);
  TEST_ASSERT_EQUAL_INT32(-89, r.mag_z);
}

static void test_negative_heading(void) {
  HeadingReading r;
  // Heading range is documented as ±180° in some contexts (raw sensor
  // range), even though the firmware normalizes to 0-360 later.
  TEST_ASSERT_TRUE(ParseHWT3100Line("Magx=1,y=2,z=3,w=-45.6", &r));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -45.6f, r.heading);
}

static void test_trailing_crlf(void) {
  HeadingReading r;
  TEST_ASSERT_TRUE(ParseHWT3100Line("Magx=1,y=2,z=3,w=4.5\r\n", &r));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.5f, r.heading);
}

static void test_trailing_cr_only(void) {
  HeadingReading r;
  TEST_ASSERT_TRUE(ParseHWT3100Line("Magx=1,y=2,z=3,w=4.5\r", &r));
}

static void test_leading_whitespace(void) {
  HeadingReading r;
  TEST_ASSERT_TRUE(ParseHWT3100Line("   Magx=1,y=2,z=3,w=4.5", &r));
}

static void test_integer_heading(void) {
  HeadingReading r;
  TEST_ASSERT_TRUE(ParseHWT3100Line("Magx=1,y=2,z=3,w=90", &r));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 90.0f, r.heading);
}

static void test_missing_field_rejected(void) {
  HeadingReading r;
  TEST_ASSERT_FALSE(ParseHWT3100Line("Magx=1,y=2,z=3", &r));
}

static void test_empty_line_rejected(void) {
  HeadingReading r;
  TEST_ASSERT_FALSE(ParseHWT3100Line("", &r));
}

static void test_garbage_line_rejected(void) {
  HeadingReading r;
  TEST_ASSERT_FALSE(ParseHWT3100Line("not a valid line at all", &r));
}

static void test_wrong_prefix_rejected(void) {
  HeadingReading r;
  // Guards against accidentally matching a differently-shaped line
  // (e.g. an AT command echo/reply) as sensor data.
  TEST_ASSERT_FALSE(ParseHWT3100Line("OK", &r));
}

static void test_null_arguments_rejected(void) {
  HeadingReading r;
  TEST_ASSERT_FALSE(ParseHWT3100Line(nullptr, &r));
  TEST_ASSERT_FALSE(ParseHWT3100Line("Magx=1,y=2,z=3,w=4.5", nullptr));
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_well_formed_positive_values);
  RUN_TEST(test_negative_mag_values);
  RUN_TEST(test_negative_heading);
  RUN_TEST(test_trailing_crlf);
  RUN_TEST(test_trailing_cr_only);
  RUN_TEST(test_leading_whitespace);
  RUN_TEST(test_integer_heading);
  RUN_TEST(test_missing_field_rejected);
  RUN_TEST(test_empty_line_rejected);
  RUN_TEST(test_garbage_line_rejected);
  RUN_TEST(test_wrong_prefix_rejected);
  RUN_TEST(test_null_arguments_rejected);
  return UNITY_END();
}
