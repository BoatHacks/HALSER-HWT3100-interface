#include "serial_terminal.h"

namespace halser {

void SerialTerminal::AddLine(const HWT3100RawLine& line) {
  lines_[next_] = line;
  next_ = (next_ + 1) % kBufferSize;
  if (count_ < kBufferSize) count_++;
}

bool SerialTerminal::to_json(JsonObject& config) {
  JsonArray lines = config["lines"].to<JsonArray>();
  // Oldest-first: once the buffer has wrapped, the oldest entry is at
  // next_ (the slot about to be overwritten); before that, it's index 0.
  size_t start_index = (count_ < kBufferSize) ? 0 : next_;
  for (size_t i = 0; i < count_; i++) {
    size_t idx = (start_index + i) % kBufferSize;
    lines.add(lines_[idx].text);
  }
  return true;
}

}  // namespace halser
