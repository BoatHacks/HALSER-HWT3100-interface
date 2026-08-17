#ifndef HALSER_SRC_HWT3100_FILTER_COMMAND_H_
#define HALSER_SRC_HWT3100_FILTER_COMMAND_H_

#include <cstddef>
#include <cstdint>

namespace halser {

// Formats an AT+FILT command for the HWT3100's output smoothing filter
// (SPEC.md §8.2a, per the manual's AT-command table): "AT+FILT=<n>\r\n"
// clamped to [0, 999]. AT+FILT=0 closes/disables the filter; AT+FILT=<1-999>
// sets the filter strength (smaller = more smoothing). There is no
// AT+FILT=1000 variant in the documented command set. Returns the
// clamped value actually written into the command, so the caller can
// persist/display what was really sent rather than a possibly-out-of-range
// input.
uint16_t FormatFilterCommand(int value, char* buf, size_t buf_len);

}  // namespace halser

#endif  // HALSER_SRC_HWT3100_FILTER_COMMAND_H_
