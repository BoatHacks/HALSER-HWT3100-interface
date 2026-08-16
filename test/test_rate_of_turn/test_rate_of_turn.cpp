#include <unity.h>

#include "rate_of_turn.h"

using halser::RateOfTurnEstimator;

void setUp(void) {}
void tearDown(void) {}

static void test_not_enough_samples(void) {
  RateOfTurnEstimator estimator(2000, 500);
  float rate = 0.0f;
  TEST_ASSERT_FALSE(estimator.GetRateOfTurn(&rate));

  estimator.AddSample(10.0f, 1000);
  TEST_ASSERT_FALSE(estimator.GetRateOfTurn(&rate));
}

static void test_span_below_minimum_rejected(void) {
  RateOfTurnEstimator estimator(2000, 500);
  estimator.AddSample(10.0f, 1000);
  estimator.AddSample(15.0f, 1200);  // only 200ms apart, min_span is 500ms
  float rate = 0.0f;
  TEST_ASSERT_FALSE(estimator.GetRateOfTurn(&rate));
}

static void test_steady_heading_is_near_zero(void) {
  RateOfTurnEstimator estimator(2000, 500);
  estimator.AddSample(90.0f, 0);
  estimator.AddSample(90.0f, 500);
  estimator.AddSample(90.0f, 1000);
  float rate = 0.0f;
  TEST_ASSERT_TRUE(estimator.GetRateOfTurn(&rate));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, rate);
}

static void test_clockwise_turn_is_positive(void) {
  RateOfTurnEstimator estimator(2000, 500);
  estimator.AddSample(0.0f, 0);
  estimator.AddSample(90.0f, 1000);  // 90 deg in 1s, turning to starboard
  float rate = 0.0f;
  TEST_ASSERT_TRUE(estimator.GetRateOfTurn(&rate));
  // 90 deg/s = pi/2 rad/s ~= 1.5708
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.5708f, rate);
}

static void test_counterclockwise_turn_is_negative(void) {
  RateOfTurnEstimator estimator(2000, 500);
  estimator.AddSample(90.0f, 0);
  estimator.AddSample(0.0f, 1000);  // turning to port
  float rate = 0.0f;
  TEST_ASSERT_TRUE(estimator.GetRateOfTurn(&rate));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.5708f, rate);
}

static void test_wraparound_clockwise_through_north(void) {
  RateOfTurnEstimator estimator(2000, 500);
  estimator.AddSample(350.0f, 0);
  estimator.AddSample(10.0f, 1000);  // 350 -> 10 is +20 deg, not -340
  float rate = 0.0f;
  TEST_ASSERT_TRUE(estimator.GetRateOfTurn(&rate));
  TEST_ASSERT_TRUE(rate > 0.0f);
  // 20 deg/s = 0.349 rad/s
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.349f, rate);
}

static void test_wraparound_counterclockwise_through_north(void) {
  RateOfTurnEstimator estimator(2000, 500);
  estimator.AddSample(10.0f, 0);
  estimator.AddSample(350.0f, 1000);  // 10 -> 350 is -20 deg, not +340
  float rate = 0.0f;
  TEST_ASSERT_TRUE(estimator.GetRateOfTurn(&rate));
  TEST_ASSERT_TRUE(rate < 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.349f, rate);
}

static void test_uses_oldest_and_newest_within_window(void) {
  // Window is 2000ms. Samples older than (newest - 2000ms) should be
  // ignored even though they're still physically in the ring buffer.
  RateOfTurnEstimator estimator(2000, 500);
  estimator.AddSample(0.0f, 0);      // outside the window once t=3000 arrives
  estimator.AddSample(0.0f, 1500);   // this becomes "oldest in window" at t=3000
  estimator.AddSample(45.0f, 3000);  // newest
  float rate = 0.0f;
  TEST_ASSERT_TRUE(estimator.GetRateOfTurn(&rate));
  // Expected: (45 - 0) deg over (3000 - 1500) = 1500ms = 1.5s
  // = 30 deg/s = 0.5236 rad/s, NOT 45 deg over 3s = 15 deg/s.
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5236f, rate);
}

static void test_startup_window_clamp_does_not_underflow(void) {
  // newest.timestamp_ms (100) < window_ms_ (2000): window_start_ms must
  // clamp to 0, not underflow to a huge unsigned value that would reject
  // every sample as "too old."
  RateOfTurnEstimator estimator(2000, 50);
  estimator.AddSample(0.0f, 0);
  estimator.AddSample(10.0f, 100);
  float rate = 0.0f;
  TEST_ASSERT_TRUE(estimator.GetRateOfTurn(&rate));
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_not_enough_samples);
  RUN_TEST(test_span_below_minimum_rejected);
  RUN_TEST(test_steady_heading_is_near_zero);
  RUN_TEST(test_clockwise_turn_is_positive);
  RUN_TEST(test_counterclockwise_turn_is_negative);
  RUN_TEST(test_wraparound_clockwise_through_north);
  RUN_TEST(test_wraparound_counterclockwise_through_north);
  RUN_TEST(test_uses_oldest_and_newest_within_window);
  RUN_TEST(test_startup_window_clamp_does_not_underflow);
  return UNITY_END();
}
