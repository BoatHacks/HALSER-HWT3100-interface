#include "hwt3100_filter_command.h"

#include <cstdio>

namespace halser {

uint16_t FormatFilterCommand(int value, char* buf, size_t buf_len) {
  int clamped = value;
  if (clamped < 0) clamped = 0;
  if (clamped > 1000) clamped = 1000;
  snprintf(buf, buf_len, "AT+FILT=%d\r\n", clamped);
  return static_cast<uint16_t>(clamped);
}

}  // namespace halser
