#ifndef HALSER_SRC_HWT3100_CALIBRATION_REPLY_H_
#define HALSER_SRC_HWT3100_CALIBRATION_REPLY_H_

namespace halser {

// The three plain-text lines the HWT3100 sends in response to
// AT+CALI=1/0/2 (SPEC.md §8.2, manual's AT-command table). Unlike
// AT+PRATE=?'s "+PRATE=<n>" reply, these aren't a structured
// "+CALI=..." format — the module just echoes a human-readable status
// string.
extern const char kCalibrationStartReply[];
extern const char kCalibrationEndReply[];
extern const char kCalibrationClearReply[];

// Returns true if `line` is exactly one of the three known calibration
// reply strings above — used to pick calibration replies out of the raw
// HWT3100 line stream, which is overwhelmingly ordinary heading data
// (see ParsePrateReply's doc comment for the same distinction).
bool IsCalibrationReply(const char* line);

}  // namespace halser

#endif  // HALSER_SRC_HWT3100_CALIBRATION_REPLY_H_
