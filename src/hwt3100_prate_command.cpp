#include "hwt3100_prate_command.h"

#include <cstdio>

namespace halser {

uint16_t FormatPrateCommand(int value, char* buf, size_t buf_len) {
  int clamped = value;
  if (clamped <= 0) {
    clamped = 0;
  } else if (clamped < 10) {
    clamped = 10;
  } else if (clamped > 10000) {
    clamped = 10000;
  }
  snprintf(buf, buf_len, "AT+PRATE=%d\r\n", clamped);
  return static_cast<uint16_t>(clamped);
}

bool ParsePrateReply(const char* line, int* out) {
  if (line == nullptr || out == nullptr) return false;

  int value = 0;
  if (sscanf(line, " +PRATE=%d", &value) != 1) return false;

  *out = value;
  return true;
}

}  // namespace halser
