#ifndef HALSER_SRC_MAGNETIC_VARIATION_LISTENER_H_
#define HALSER_SRC_MAGNETIC_VARIATION_LISTENER_H_

#include <N2kMessages.h>
#include <NMEA2000.h>

#include "n2k_senders.h"

namespace halser {

// Listens for PGN 127258 (Magnetic Variation) from another N2K device
// (typically a GPS or chartplotter) and feeds it into an
// externally-owned ExpiringValue (SPEC.md §5.1a) — this firmware has no
// GPS or geomagnetic model of its own, so it can't originate a
// variation value, only relay one that already exists on the bus.
//
// Deliberately one-directional: this class only ever receives PGN
// 127258, it never constructs or sends one (ARCHITECTURE.md §2.4b).
class MagneticVariationListener : public tNMEA2000::tMsgHandler {
 public:
  MagneticVariationListener(tNMEA2000* nmea2000, ExpiringValue<float>* variation)
      : tNMEA2000::tMsgHandler(127258L, nmea2000), variation_(variation) {}

  void HandleMsg(const tN2kMsg& msg) override;

 private:
  ExpiringValue<float>* variation_;
};

}  // namespace halser

#endif  // HALSER_SRC_MAGNETIC_VARIATION_LISTENER_H_
