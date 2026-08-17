#include <cstring>

#include <unity.h>

#include "hwt3100_uart_command.h"

using halser::FormatUartCommand;
using halser::kBaud115200;
using halser::kBaud460800;
using halser::kBaud9600;

void setUp(void) {}
void tearDown(void) {}

static void test_exact_9600(void) {
  char buf[24];
  int actual = FormatUartCommand(kBaud9600, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_INT(kBaud9600, actual);
  TEST_ASSERT_EQUAL_STRING("AT+UART=0\r\n", buf);
}

static void test_exact_115200(void) {
  char buf[24];
  int actual = FormatUartCommand(kBaud115200, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_INT(kBaud115200, actual);
  TEST_ASSERT_EQUAL_STRING("AT+UART=1\r\n", buf);
}

static void test_exact_460800(void) {
  char buf[24];
  int actual = FormatUartCommand(kBaud460800, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_INT(kBaud460800, actual);
  TEST_ASSERT_EQUAL_STRING("AT+UART=2\r\n", buf);
}

static void test_snaps_to_nearest_low(void) {
  // 50000 is closer to 9600 (delta 40400) than to 115200 (delta 65200).
  char buf[24];
  int actual = FormatUartCommand(50000, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_INT(kBaud9600, actual);
  TEST_ASSERT_EQUAL_STRING("AT+UART=0\r\n", buf);
}

static void test_snaps_to_nearest_mid(void) {
  // 200000 is closer to 115200 (delta 84800) than to 460800 (delta 260800).
  char buf[24];
  int actual = FormatUartCommand(200000, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_INT(kBaud115200, actual);
  TEST_ASSERT_EQUAL_STRING("AT+UART=1\r\n", buf);
}

static void test_tie_breaks_toward_lower_9600_115200(void) {
  // Midpoint between 9600 and 115200 is 62400 - exact tie.
  char buf[24];
  int actual = FormatUartCommand(62400, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_INT(kBaud9600, actual);
  TEST_ASSERT_EQUAL_STRING("AT+UART=0\r\n", buf);
}

static void test_tie_breaks_toward_lower_115200_460800(void) {
  // Midpoint between 115200 and 460800 is 288000 - exact tie.
  char buf[24];
  int actual = FormatUartCommand(288000, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_INT(kBaud115200, actual);
  TEST_ASSERT_EQUAL_STRING("AT+UART=1\r\n", buf);
}

static void test_zero_snaps_to_9600(void) {
  char buf[24];
  int actual = FormatUartCommand(0, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_INT(kBaud9600, actual);
  TEST_ASSERT_EQUAL_STRING("AT+UART=0\r\n", buf);
}

static void test_very_large_snaps_to_460800(void) {
  char buf[24];
  int actual = FormatUartCommand(2000000, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_INT(kBaud460800, actual);
  TEST_ASSERT_EQUAL_STRING("AT+UART=2\r\n", buf);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_exact_9600);
  RUN_TEST(test_exact_115200);
  RUN_TEST(test_exact_460800);
  RUN_TEST(test_snaps_to_nearest_low);
  RUN_TEST(test_snaps_to_nearest_mid);
  RUN_TEST(test_tie_breaks_toward_lower_9600_115200);
  RUN_TEST(test_tie_breaks_toward_lower_115200_460800);
  RUN_TEST(test_zero_snaps_to_9600);
  RUN_TEST(test_very_large_snaps_to_460800);
  return UNITY_END();
}
