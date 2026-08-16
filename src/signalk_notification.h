#ifndef HALSER_SRC_SIGNALK_NOTIFICATION_H_
#define HALSER_SRC_SIGNALK_NOTIFICATION_H_

#include <ArduinoJson.h>

#include "sensesp/signalk/signalk_emitter.h"

namespace halser {

// Minimal SignalK notification emitter (SPEC.md §6). SensESP 3.2.0 has
// no built-in "send a notification" helper — only SKPrefixListener, for
// *receiving* notifications.* — so this subclasses sensesp::SKEmitter
// directly, the same generic extension point sensesp::SKOutput<T> itself
// uses (SKDeltaQueue sweeps SKEmitter::get_sources() unconditionally and
// calls as_signalk_json() on each; verified against the actual vendored
// source, not assumed — see docs/plans/fault-indication.md).
//
// Only "normal"/"alarm" states are used (SPEC.md §6 is binary: stale or
// not), not the full SignalK alarm-state enum (nominal/alert/warn/alarm/
// emergency).
class SKNotification : public sensesp::SKEmitter {
 public:
  explicit SKNotification(const String& sk_path) : sensesp::SKEmitter(sk_path) {}

  void Set(const String& state, const String& message) {
    state_ = state;
    message_ = message;
  }

  void as_signalk_json(JsonDocument& doc) override {
    doc["path"] = get_sk_path();
    JsonObject value = doc["value"].to<JsonObject>();
    value["state"] = state_;
    value["message"] = message_;
  }

 private:
  String state_ = "normal";
  String message_ = "";
};

}  // namespace halser

#endif  // HALSER_SRC_SIGNALK_NOTIFICATION_H_
