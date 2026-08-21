#include "hwt3100_parser.h"

#include <cstdio>

bool ParseHWT3100Line(const char* line, HeadingReading* out) {
  if (line == nullptr || out == nullptr) return false;

  int mag_x = 0, mag_y = 0, mag_z = 0;
  float heading = 0.0f;

  // Leading whitespace in the format string matches any amount (incl.
  // none) of whitespace in the input, so this tolerates stray
  // leading spaces without special-casing them. Anything after the
  // matched fields (e.g. a trailing \r) is simply left unconsumed,
  // which is fine since success is judged by field count, not by
  // consuming the whole line.
  int matched =
      sscanf(line, " Magx:%d,Magy:%d,Magz:%d,Yaw:%f", &mag_x, &mag_y, &mag_z,
             &heading);
  if (matched != 4) return false;

  out->mag_x = mag_x;
  out->mag_y = mag_y;
  out->mag_z = mag_z;
  out->heading = heading;
  return true;
}
