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

static void test_regression_uses_all_samples_not_just_endpoints(void) {
  // A pure two-point method (endpoints only) would give (20-0)/1000ms =
  // 20 deg/s regardless of the middle sample. The windowed
  // least-squares fit should be pulled away from that by the off-line
  // interior point at (300ms, 15deg) -- demonstrating every buffered
  // sample participates, not just the first and last.
  RateOfTurnEstimator estimator(2000, 500);
  estimator.AddSample(0.0f, 0);
  estimator.AddSample(15.0f, 300);
  estimator.AddSample(20.0f, 1000);
  float rate = 0.0f;
  TEST_ASSERT_TRUE(estimator.GetRateOfTurn(&rate));
  // Least-squares slope for these 3 points works out to ~17.72 deg/s
  // (0.3093 rad/s) -- meaningfully different from the naive endpoints-
  // only 20 deg/s (0.3491 rad/s), confirming the interior sample moved
  // the estimate.
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.3093f, rate);
  TEST_ASSERT_TRUE(fabsf(rate - 0.3491f) > 0.02f);
}

static void test_wraparound_regression_multiple_samples(void) {
  // Three collinear samples turning through the 0/360 boundary: 350 ->
  // 365(=5) -> 380(=20), each step +15 deg. Verifies unwrapping is
  // applied per-step across the whole window, not just at the two
  // endpoints -- a naive fit on raw (unwrapped) values would see
  // 350 -> 5 -> 20 and badly misread the first step as -345 deg.
  RateOfTurnEstimator estimator(2000, 500);
  estimator.AddSample(350.0f, 0);
  estimator.AddSample(5.0f, 500);
  estimator.AddSample(20.0f, 1000);
  float rate = 0.0f;
  TEST_ASSERT_TRUE(estimator.GetRateOfTurn(&rate));
  // 30 deg/s = 0.5236 rad/s
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5236f, rate);
}

static void test_set_window_ms_narrows_included_samples(void) {
  // Deliberately non-collinear (0, 5, 20 deg at t=0/750/1500) so
  // dropping a sample actually changes the fitted slope -- three
  // perfectly-collinear points would give the same least-squares slope
  // regardless of which subset is included, which would defeat the
  // point of this test.
  RateOfTurnEstimator estimator(2000, 100);
  estimator.AddSample(0.0f, 0);
  estimator.AddSample(5.0f, 750);
  estimator.AddSample(20.0f, 1500);
  float rate_wide_window = 0.0f;
  TEST_ASSERT_TRUE(estimator.GetRateOfTurn(&rate_wide_window));

  estimator.SetWindowMs(1000);
  float rate_narrow_window = 0.0f;
  TEST_ASSERT_TRUE(estimator.GetRateOfTurn(&rate_narrow_window));
  // Narrowing the window drops the t=0 sample (outside 1500-1000=500),
  // leaving only the two samples 750ms apart -- a different fit than
  // all three samples over 1500ms.
  TEST_ASSERT_TRUE(fabsf(rate_narrow_window - rate_wide_window) > 0.001f);
}

static void test_set_min_span_ms_rejects_narrower_spacing(void) {
  RateOfTurnEstimator estimator(2000, 100);
  estimator.AddSample(0.0f, 0);
  estimator.AddSample(10.0f, 200);  // 200ms span
  float rate = 0.0f;
  TEST_ASSERT_TRUE(estimator.GetRateOfTurn(&rate));  // 200ms >= 100ms min

  estimator.SetMinSpanMs(500);
  TEST_ASSERT_FALSE(estimator.GetRateOfTurn(&rate));  // 200ms < 500ms min
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
  RUN_TEST(test_regression_uses_all_samples_not_just_endpoints);
  RUN_TEST(test_wraparound_regression_multiple_samples);
  RUN_TEST(test_set_window_ms_narrows_included_samples);
  RUN_TEST(test_set_min_span_ms_rejects_narrower_spacing);
  return UNITY_END();
}
