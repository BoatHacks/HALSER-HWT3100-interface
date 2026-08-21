#ifndef HALSER_SRC_HWT3100_PARSER_H_
#define HALSER_SRC_HWT3100_PARSER_H_

#include "hwt3100_types.h"

// Parses one line of the HWT3100's default ASCII-mode output:
//   Magx:<int>,Magy:<int>,Magz:<int>,Yaw:<float>
// (a trailing \r, \n, or \r\n — or none — is fine; the caller is
// expected to have split the incoming stream into lines already).
//
// This is the format confirmed against a real HWT3100-TTL module's
// serial output (via the ESP_LOGD line logging in hwt3100_serial.cpp).
// An earlier version of this parser assumed a differently-punctuated
// format (`Magx=<int>,y=<int>,z=<int>,w=<float>`) sourced from the
// manual + vendor SDK example rather than a live device, which never
// actually matched real hardware output -- see CHANGELOG.md.
//
// On success, fills heading/mag_x/mag_y/mag_z in *out and returns true.
// Does NOT set out->timestamp — that's a hardware clock read the caller
// supplies, not something this pure parsing function should own (see
// docs/plans/hwt3100-serial-parser.md).
//
// On a malformed/incomplete line, returns false and leaves *out
// unmodified.
bool ParseHWT3100Line(const char* line, HeadingReading* out);

#endif  // HALSER_SRC_HWT3100_PARSER_H_
