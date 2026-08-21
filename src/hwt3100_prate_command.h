#ifndef HALSER_SRC_HWT3100_PRATE_COMMAND_H_
#define HALSER_SRC_HWT3100_PRATE_COMMAND_H_

#include <cstddef>
#include <cstdint>

namespace halser {

// Sentinel meaning "not yet known" for the persisted output-rate config
// value (gateway.cpp) — not a value ever sent to the module. Outside the
// documented AT+PRATE domain ({0} u [10, 10000]), so it can't collide
// with a real setting.
constexpr int kPrateUnknown = -1;

// Formats an AT+PRATE command (SPEC.md §8.2b, per the manual's
// AT-command table): "AT+PRATE=<n>\r\n". Valid domain is {0} u [10,
// 10000] — 0 selects single-return mode (module stops streaming
// unsolicited data; see the SPEC.md warning on why this is dangerous
// for this firmware specifically), 10-10000 sets a periodic interval in
// ms. Values in (0, 10) clamp up to 10 (the lowest valid periodic
// interval) rather than down to 0, since collapsing a small positive
// request into "stop streaming entirely" would surprise the caller far
// more than rounding up would. Returns the value actually written into
// the command.
uint16_t FormatPrateCommand(int value, char* buf, size_t buf_len);

// Parses a "+PRATE=<n>\r\n" reply line (the module's response to
// AT+PRATE=?, per the manual's query-reply convention shared with
// AT+MODE=?/AT+ID=?). Returns true and fills *out on success; returns
// false and leaves *out untouched otherwise (e.g. an ordinary
// "Magx:...,Yaw:..." data line, which is the overwhelming majority of
// what arrives on this same serial stream).
bool ParsePrateReply(const char* line, int* out);

}  // namespace halser

#endif  // HALSER_SRC_HWT3100_PRATE_COMMAND_H_
