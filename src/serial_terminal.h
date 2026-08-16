#ifndef HALSER_SRC_SERIAL_TERMINAL_H_
#define HALSER_SRC_SERIAL_TERMINAL_H_

#include <ArduinoJson.h>

#include <cstddef>

#include "hwt3100_types.h"
#include "sensesp/system/saveable.h"
#include "sensesp/system/serializable.h"

namespace halser {

// Exposes the most recent HWT3100 raw serial lines through SensESP's
// existing config REST API (SPEC.md §8.1). This is the implementation
// the "serial terminal" ended up as, after finding SensESP 3.2.0 has no
// public way to add a custom HTTP/WebSocket endpoint — see
// docs/plans/gateway-wiring.md.
//
// Deliberately NOT sensesp::FileSystemSaveable: this buffer changes on
// every incoming line, and persisting a diagnostic log to flash on every
// update would wear it out for no benefit. Plain Saveable's
// load()/save()/clear() default to no-ops, which is exactly right here.
// from_json() is also left at Serializable's default (returns false),
// making writes to this config item a no-op — a read-only view.
class SerialTerminal : public sensesp::Saveable, public sensesp::Serializable {
 public:
  explicit SerialTerminal(const String& config_path) : Saveable(config_path) {}

  // Called from the raw-line consumer set up in gateway.cpp — this
  // class doesn't read HWT3100RawLine off a TaskQueueProducer itself, to
  // keep it a plain buffer with no FreeRTOS/event-loop concerns of its
  // own (see ARCHITECTURE.md §2.5, §2.6).
  void AddLine(const HWT3100RawLine& line);

  bool to_json(JsonObject& config) override;

 private:
  static constexpr size_t kBufferSize = 30;
  HWT3100RawLine lines_[kBufferSize];
  size_t count_ = 0;  // number of valid entries, <= kBufferSize
  size_t next_ = 0;   // next slot to write, wraps
};

// ConfigItemT<T>::get_default_config_schema() unconditionally calls
// ConfigSchema(*config_object_) for every T it's instantiated with, even
// though gateway.cpp always calls set_config_schema() explicitly (which
// takes precedence at runtime) — the compiler still needs this overload
// to exist. Found via ADL since SerialTerminal lives in this namespace.
inline const String ConfigSchema(const SerialTerminal&) {
  return R"schema({"type":"object","properties":{"lines":{"title":"Lines","type":"array","items":{"type":"string"}}}})schema";
}

}  // namespace halser

#endif  // HALSER_SRC_SERIAL_TERMINAL_H_
