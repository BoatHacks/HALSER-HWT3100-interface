#ifndef HALSER_SRC_HWT3100_PARSER_H_
#define HALSER_SRC_HWT3100_PARSER_H_

#include "hwt3100_types.h"

// Parses one line of the HWT3100's default ASCII-mode output:
//   Magx=<int>,y=<int>,z=<int>,w=<float>
// (a trailing \r, \n, or \r\n — or none — is fine; the caller is
// expected to have split the incoming stream into lines already).
//
// On success, fills heading/mag_x/mag_y/mag_z in *out and returns true.
// Does NOT set out->timestamp — that's a hardware clock read the caller
// supplies, not something this pure parsing function should own (see
// docs/plans/hwt3100-serial-parser.md).
//
// On a malformed/incomplete line, returns false and leaves *out
// unmodified.
//
// See SPEC.md §1.2 for where this format comes from (the HWT3100-TTL/232
// manual + the vendor's bundled Arduino SDK example) and §11 for the
// open question of whether real hardware always matches it exactly.
bool ParseHWT3100Line(const char* line, HeadingReading* out);

#endif  // HALSER_SRC_HWT3100_PARSER_H_
