#include <unity.h>

#include "calibration_offset.h"
#include "hwt3100_types.h"

using halser::ApplyCalibrationOffset;

void setUp(void) {}
void tearDown(void) {}

static void test_zero_raw_yaw_stays_zero_with_no_offset(void) {
  // North (raw Yaw 0) must read as compass north (0), unaffected by the
  // CCW->CW conversion since negating zero is still zero.
  HeadingReading reading;
  reading.heading = 0.0f;
  HeadingReading corrected = ApplyCalibrationOffset(reading, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, corrected.heading);
}

static void test_turning_toward_west_decreases_compass_heading(void) {
  // The HWT3100's raw Yaw increases counterclockwise (confirmed on real
  // hardware): turning the module toward west from north increases raw
  // Yaw. The resulting compass heading must decrease (wrapping down
  // toward 270, west's compass bearing), not increase.
  HeadingReading at_north;
  at_north.heading = 0.0f;
  HeadingReading turned_toward_west;
  turned_toward_west.heading = 30.0f;  // raw Yaw increased, turning left

  float north_heading = ApplyCalibrationOffset(at_north, 0.0f).heading;
  float west_turn_heading = ApplyCalibrationOffset(turned_toward_west, 0.0f).heading;

  // 330 (= -30 wrapped) is "west of due north" on a compass, i.e. less
  // than 360 and reached by decreasing from 0 -- not 30.
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 330.0f, west_turn_heading);
  TEST_ASSERT_TRUE(west_turn_heading != 30.0f);
  (void)north_heading;
}

static void test_raw_yaw_at_due_west_reads_270(void) {
  // Under the CCW-positive raw convention, due west (a 90-degree
  // counterclockwise turn from north) is raw Yaw = 90. Compass bearing
  // for due west is 270.
  HeadingReading reading;
  reading.heading = 90.0f;
  HeadingReading corrected = ApplyCalibrationOffset(reading, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 270.0f, corrected.heading);
}

static void test_offset_still_applies_after_axis_conversion(void) {
  HeadingReading reading;
  reading.heading = 90.0f;  // raw Yaw -> due west (270) before offset
  HeadingReading corrected = ApplyCalibrationOffset(reading, 10.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 280.0f, corrected.heading);
}

static void test_result_always_wraps_into_zero_to_360(void) {
  HeadingReading reading;
  reading.heading = -10.0f;  // raw Yaw already negative
  HeadingReading corrected = ApplyCalibrationOffset(reading, 0.0f);
  TEST_ASSERT_TRUE(corrected.heading >= 0.0f);
  TEST_ASSERT_TRUE(corrected.heading < 360.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, corrected.heading);
}

static void test_magnetic_field_fields_pass_through_unchanged(void) {
  HeadingReading reading;
  reading.heading = 45.0f;
  reading.mag_x = 111;
  reading.mag_y = -222;
  reading.mag_z = 333;
  HeadingReading corrected = ApplyCalibrationOffset(reading, 5.0f);
  TEST_ASSERT_EQUAL_INT32(111, corrected.mag_x);
  TEST_ASSERT_EQUAL_INT32(-222, corrected.mag_y);
  TEST_ASSERT_EQUAL_INT32(333, corrected.mag_z);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_zero_raw_yaw_stays_zero_with_no_offset);
  RUN_TEST(test_turning_toward_west_decreases_compass_heading);
  RUN_TEST(test_raw_yaw_at_due_west_reads_270);
  RUN_TEST(test_offset_still_applies_after_axis_conversion);
  RUN_TEST(test_result_always_wraps_into_zero_to_360);
  RUN_TEST(test_magnetic_field_fields_pass_through_unchanged);
  return UNITY_END();
}
