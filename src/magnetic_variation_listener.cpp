#include "magnetic_variation_listener.h"

namespace halser {

void MagneticVariationListener::HandleMsg(const tN2kMsg& msg) {
  unsigned char sid;
  tN2kMagneticVariation source;
  uint16_t days_since_1970;
  double variation_rad;

  if (!ParseN2kMagneticVariation(msg, sid, source, days_since_1970,
                                  variation_rad)) {
    return;
  }
  if (N2kIsNA(variation_rad)) {
    return;
  }

  variation_->update(static_cast<float>(variation_rad));
}

}  // namespace halser
