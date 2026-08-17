#ifndef HALSER_SRC_HWT3100_FILTER_COMMAND_H_
#define HALSER_SRC_HWT3100_FILTER_COMMAND_H_

#include <cstddef>
#include <cstdint>

namespace halser {

// Formats an AT+FILT command for the HWT3100's output smoothing filter
// (SPEC.md §8.2, per the HWT3100-TTL/232 manual §5.3.1): "AT+FILT=<n>\r\n"
// clamped to [0, 1000]. 1-999 sets the filter strength (smaller = more
// smoothing); 0 and 1000 are both valid commands to send but the manual
// documents them as "not to set the filter" (i.e. no smoothing applied,
// matching the module's own default of 0). Returns the clamped value
// actually written into the command, so the caller can persist/display
// what was really sent rather than a possibly-out-of-range input.
uint16_t FormatFilterCommand(int value, char* buf, size_t buf_len);

}  // namespace halser

#endif  // HALSER_SRC_HWT3100_FILTER_COMMAND_H_
