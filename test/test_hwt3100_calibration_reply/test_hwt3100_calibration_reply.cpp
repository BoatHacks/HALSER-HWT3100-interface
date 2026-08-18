#include <unity.h>

#include "hwt3100_calibration_reply.h"

using halser::IsCalibrationReply;
using halser::kCalibrationClearReply;
using halser::kCalibrationEndReply;
using halser::kCalibrationStartReply;

void setUp(void) {}
void tearDown(void) {}

static void test_recognizes_start_reply(void) {
  TEST_ASSERT_TRUE(IsCalibrationReply(kCalibrationStartReply));
  TEST_ASSERT_TRUE(IsCalibrationReply("Calibrating"));
}

static void test_recognizes_end_reply(void) {
  TEST_ASSERT_TRUE(IsCalibrationReply(kCalibrationEndReply));
  TEST_ASSERT_TRUE(IsCalibrationReply("Calibration completed"));
}

static void test_recognizes_clear_reply(void) {
  TEST_ASSERT_TRUE(IsCalibrationReply(kCalibrationClearReply));
  TEST_ASSERT_TRUE(IsCalibrationReply("Reset mag offset param"));
}

static void test_rejects_ordinary_data_line(void) {
  // The overwhelming majority of lines on this stream are heading data,
  // not command replies - these must not be misparsed as a calibration
  // reply.
  TEST_ASSERT_FALSE(IsCalibrationReply("Magx=1,y=2,z=3,w=4.5"));
}

static void test_rejects_partial_match(void) {
  TEST_ASSERT_FALSE(IsCalibrationReply("Calibrating now"));
  TEST_ASSERT_FALSE(IsCalibrationReply("Calibration"));
}

static void test_rejects_null(void) {
  TEST_ASSERT_FALSE(IsCalibrationReply(nullptr));
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_recognizes_start_reply);
  RUN_TEST(test_recognizes_end_reply);
  RUN_TEST(test_recognizes_clear_reply);
  RUN_TEST(test_rejects_ordinary_data_line);
  RUN_TEST(test_rejects_partial_match);
  RUN_TEST(test_rejects_null);
  return UNITY_END();
}
