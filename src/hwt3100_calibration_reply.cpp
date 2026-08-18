#include "hwt3100_calibration_reply.h"

#include <cstring>

namespace halser {

const char kCalibrationStartReply[] = "Calibrating";
const char kCalibrationEndReply[] = "Calibration completed";
const char kCalibrationClearReply[] = "Reset mag offset param";

bool IsCalibrationReply(const char* line) {
  if (line == nullptr) return false;
  return strcmp(line, kCalibrationStartReply) == 0 ||
         strcmp(line, kCalibrationEndReply) == 0 ||
         strcmp(line, kCalibrationClearReply) == 0;
}

}  // namespace halser
