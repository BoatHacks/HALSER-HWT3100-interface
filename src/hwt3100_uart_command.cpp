#include "hwt3100_uart_command.h"

#include <cstdio>

namespace halser {

int FormatUartCommand(int requested_baud, char* buf, size_t buf_len) {
  const int candidates[3] = {kBaud9600, kBaud115200, kBaud460800};

  int best_index = 0;
  long best_distance = -1;
  for (int i = 0; i < 3; i++) {
    long distance = requested_baud > candidates[i]
                         ? static_cast<long>(requested_baud) - candidates[i]
                         : static_cast<long>(candidates[i]) - requested_baud;
    // Strictly-less comparison keeps the first (lowest) candidate at an
    // exact tie, since candidates are in ascending order.
    if (best_distance < 0 || distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }

  snprintf(buf, buf_len, "AT+UART=%d\r\n", best_index);
  return candidates[best_index];
}

}  // namespace halser
