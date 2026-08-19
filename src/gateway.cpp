#include "gateway.h"

#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <esp_mac.h>

#include "calibration_offset.h"
#include "halser_const.h"
#include "hwt3100_calibration_commands.h"
#include "hwt3100_calibration_reply.h"
#include "hwt3100_prate_command.h"
#include "hwt3100_serial.h"
#include "hwt3100_types.h"
#include "hwt3100_uart_command.h"
#include "magnetic_variation_listener.h"
#include "mfd_calibration_bridge.h"
#include "n2k_senders.h"
#include "rate_of_turn.h"
#include "sensesp/signalk/signalk_metadata.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/observablevalue.h"
#include "sensesp/system/task_queue_producer.h"
#include "sensesp/ui/config_item.h"
#include "sensesp/ui/status_page_item.h"
#include "sensesp/ui/ui_button.h"
#include "sensesp_app_builder.h"
#include "serial_terminal.h"

using namespace sensesp;

namespace {

tNMEA2000* nmea2000 = nullptr;

// PGNs this firmware actually transmits, beyond the NMEA2000-library's
// own boilerplate (address claim, heartbeat, product/config info,
// which it reports automatically). Passed to ExtendTransmitMessages()
// so PGN 126464 ("PGN List - Transmit") queries — and any MFD/tool
// that uses that list to decide what data sources a device offers —
// see 127250/127251 too, not just the boilerplate set. 0-terminated
// per the library's own convention; must outlive the call (the library
// stores the pointer, not a copy), hence file-scope rather than local.
const unsigned long kTransmitMessages[] PROGMEM = {127250L, 127251L, 0};

// Rate-of-turn sliding window (SPEC.md §11 flags these as reasonable
// defaults needing real-hardware tuning, not values derived from an
// actual helm/autopilot's sensitivity requirements).
constexpr unsigned long kRateOfTurnWindowMs = 2000;
constexpr unsigned long kRateOfTurnMinSpanMs = 500;

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegreesToRadians = kPi / 180.0f;

// Matches N2kHeadingSender's ExpiringValue expiry (SPEC.md §6, §10): the
// SignalK meta.timeout advisory should agree with when N2K actually
// starts sending "not available."
constexpr float kHeadingTimeoutSeconds = 5.0f;

// Matches N2kHeadingSender's variation_ expiry default (SPEC.md §5.1a)
// — much longer than heading's, since bus-sourced magnetic variation
// changes on a geographic timescale and typically isn't rebroadcast
// every second the way heading is.
constexpr float kVariationTimeoutSeconds = 300.0f;
constexpr float kTwoPi = 2.0f * kPi;

// Baud auto-detection order and per-candidate timeout (SPEC.md §8.2c):
// recommended rate first (fast path for an already-configured module),
// then the module's factory default, then the fastest supported rate.
// 1s/candidate is a reasonable guess at how long it takes to see at
// least one full line at the module's documented output rates — not
// derived from real-hardware timing (SPEC.md §11).
constexpr int kBaudCandidates[] = {halser::kBaud115200, halser::kBaud9600,
                                    halser::kBaud460800};
constexpr size_t kNumBaudCandidates =
    sizeof(kBaudCandidates) / sizeof(kBaudCandidates[0]);
constexpr unsigned long kBaudDetectTimeoutMs = 1000;

/// Used for SetDeviceInformation()'s "unique number" — deliberately NOT
/// the Precision-9 reference's hardcoded value (SPEC.md §10), so that
/// two devices running this firmware don't collide on the same N2K bus.
static uint32_t GetBoardSerialNumber() {
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  return (mac[3] << 16) | (mac[4] << 8) | mac[5];
}

}  // namespace

void run_hwt3100_gateway() {
  Serial.setTxTimeoutMs(0);
  SetupLogging(ESP_LOG_DEBUG);

  // SensESP application
  SensESPAppBuilder builder;
  auto sensesp_app = (&builder)
                          ->set_hostname("halser-hwt3100")
                          ->set_button_pin(kButtonPin)
                          ->enable_ota("halser-hwt3100")
                          ->enable_system_info_sensors()
                          ->get_app();

  // No separate RGB LED use here — SensESP's own RGBSystemStatusLed
  // (auto-instantiated from the PIN_RGB_LED build flag) already owns
  // GPIO8 to show WiFi/WebSocket connection status, with no public hook
  // to share or override it. Fault indication (SPEC.md §6) is
  // SignalK-notification-only; see docs/plans/fault-indication.md.

  // --- Configuration (SPEC.md §7) ---

  auto n2k_enabled = std::make_shared<PersistingObservableValue<bool>>(
      true, "/n2k/enabled");
  ConfigItem(n2k_enabled)
      ->set_title("Enable NMEA 2000 Output")
      ->set_sort_order(100)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  auto n2k_heading_pgn_enabled = std::make_shared<PersistingObservableValue<bool>>(
      true, "/n2k/heading_pgn_enabled");
  ConfigItem(n2k_heading_pgn_enabled)
      ->set_title("Enable PGN 127250 (Vessel Heading)")
      ->set_sort_order(110)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  auto n2k_rate_of_turn_pgn_enabled = std::make_shared<PersistingObservableValue<bool>>(
      true, "/n2k/rate_of_turn_pgn_enabled");
  ConfigItem(n2k_rate_of_turn_pgn_enabled)
      ->set_title("Enable PGN 127251 (Rate of Turn)")
      ->set_description(
          "Computed from a sliding window of heading readings (SPEC.md §5.1) — the HWT3100 has no gyroscope.")
      ->set_sort_order(111)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  auto signalk_enabled = std::make_shared<PersistingObservableValue<bool>>(
      true, "/signalk/enabled");
  ConfigItem(signalk_enabled)
      ->set_title("Enable SignalK Output")
      ->set_sort_order(120)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  // Per-delta toggles (SPEC.md §5.2, §7), same pattern as the N2K
  // per-PGN toggles above — each independently gated on top of the
  // signalk_enabled master switch, not a replacement for it.
  auto sk_heading_enabled = std::make_shared<PersistingObservableValue<bool>>(
      true, "/signalk/heading_enabled");
  ConfigItem(sk_heading_enabled)
      ->set_title("Enable navigation.headingMagnetic")
      ->set_sort_order(121)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  auto sk_rate_of_turn_enabled = std::make_shared<PersistingObservableValue<bool>>(
      true, "/signalk/rate_of_turn_enabled");
  ConfigItem(sk_rate_of_turn_enabled)
      ->set_title("Enable navigation.rateOfTurn")
      ->set_description(
          "Computed from a sliding window of heading readings (SPEC.md §5.1) — the HWT3100 has no gyroscope.")
      ->set_sort_order(122)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  auto sk_heading_true_enabled = std::make_shared<PersistingObservableValue<bool>>(
      true, "/signalk/heading_true_enabled");
  ConfigItem(sk_heading_true_enabled)
      ->set_title("Enable navigation.headingTrue")
      ->set_description(
          "Computed as magnetic heading + variation (SPEC.md §5.1a). Only "
          "published when a recent magnetic variation has been seen from "
          "another device on the N2K bus (PGN 127258) — this firmware has "
          "no GPS or geomagnetic model of its own.")
      ->set_sort_order(123)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  // Raw magnetic field as SignalK deltas (SPEC.md §5.2) — diagnostic
  // data with no established SignalK path, so it's off by default and
  // gated separately from signalk_enabled (both must be true to
  // publish).
  auto raw_mag_field_enabled = std::make_shared<PersistingObservableValue<bool>>(
      false, "/signalk/raw_mag_field_enabled");
  ConfigItem(raw_mag_field_enabled)
      ->set_title("Enable Raw Magnetic Field SignalK Output")
      ->set_description(
          "Publishes sensors.hwt3100.magneticField.x/y/z -- raw, "
          "uncalibrated sensor counts from the HWT3100 (SPEC.md §5.2). "
          "Diagnostic-only; off by default.")
      ->set_sort_order(124)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Enabled","type":"boolean"}}})schema");

  auto heading_offset = std::make_shared<PersistingObservableValue<float>>(
      0.0f, "/calibration/heading_offset");
  ConfigItem(heading_offset)
      ->set_title("Heading Calibration Offset")
      ->set_description(
          "Degrees added to the raw HWT3100 heading to correct for mounting misalignment")
      ->set_sort_order(50)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Offset (degrees)","type":"number"}}})schema");

  // AT+FILT (SPEC.md §8.2a): on-module output smoothing filter.
  // AT+FILT=0 closes the filter (module default); AT+FILT=<1-999> sets
  // the filter strength (smaller = smoother). Persisted so a reboot
  // re-applies it (module has no way to report its current setting
  // back to us).
  auto output_filter = std::make_shared<PersistingObservableValue<int>>(
      0, "/hwt3100/output_filter");
  ConfigItem(output_filter)
      ->set_title("HWT3100 Output Filter (AT+FILT)")
      ->set_description(
          "On-module smoothing filter. 0 = off/closed (module default). "
          "1-999 = filter strength; smaller values smooth more.")
      ->set_sort_order(55)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Filter","type":"integer","minimum":0,"maximum":999}}})schema");

  // AT+PRATE (SPEC.md §8.2b): on-module output data rate. Persisted so
  // a reboot re-applies it, same as output_filter above — except this
  // one starts life unknown (halser::kPrateUnknown = -1, outside the
  // real {0} u [10,10000] domain), since forcing an arbitrary default
  // here could be actively harmful: AT+PRATE=0 puts the module into
  // single-return mode, silencing the continuous stream this firmware's
  // entire read pipeline depends on. Startup wiring below queries the
  // module instead of guessing whenever this is still kPrateUnknown.
  auto output_prate = std::make_shared<PersistingObservableValue<int>>(
      halser::kPrateUnknown, "/hwt3100/output_prate");
  ConfigItem(output_prate)
      ->set_title("HWT3100 Output Rate (AT+PRATE, ms)")
      ->set_description(
          "Module's own output interval in ms. -1 = not yet known; queried "
          "from the sensor automatically at boot. 10-10000 = periodic "
          "interval in ms -- 100 (10 datagrams/second) is the recommended "
          "minimum for usable heading/rate-of-turn resolution; going lower "
          "gives faster updates at the cost of more UART/CPU load. 0 = "
          "single-return mode -- WARNING: this disables the continuous "
          "data stream this firmware depends on; only set this if you "
          "understand the consequences.")
      ->set_sort_order(56)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Rate (ms)","type":"integer","minimum":-1,"maximum":10000}}})schema");

  // AT+UART (SPEC.md §8.2c): the UART baud rate itself, not just a
  // module setting -- unlike output_filter/output_prate above, changing
  // this also has to reconfigure this firmware's own Serial1, not just
  // send a command. Starts unknown (halser::kBaudUnknown = -1); startup
  // wiring below auto-detects it by trying candidate rates in turn
  // rather than assuming, since a wrong assumption here means no data
  // ever arrives at all (not just a suboptimal setting).
  auto hwt3100_baud = std::make_shared<PersistingObservableValue<int>>(
      halser::kBaudUnknown, "/hwt3100/baud");
  ConfigItem(hwt3100_baud)
      ->set_title("HWT3100 UART Baud Rate")
      ->set_description(
          "-1 = not yet known; auto-detected at boot by trying 115200 "
          "(recommended), 9600 (factory default), then 460800 in turn. "
          "Set to 9600, 115200, or 460800 to send AT+UART and switch the "
          "module's baud live -- the firmware reconfigures its own UART "
          "to match immediately after (values are snapped to the nearest "
          "of these three).")
      ->set_sort_order(58)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Baud","type":"integer"}}})schema");

  // Rate-of-turn window/min-span (SPEC.md §7, §11): previously
  // compile-time constants, now tunable without a firmware rebuild —
  // the right smoothing-vs-responsiveness trade-off is a per-
  // installation judgment call. Plain defaults, applied at boot and on
  // every change (no unknown-sentinel/discovery needed, unlike the
  // AT+ config items above — this is a firmware-side number with no
  // hardware round-trip).
  auto rate_of_turn_window_ms = std::make_shared<PersistingObservableValue<int>>(
      static_cast<int>(kRateOfTurnWindowMs), "/rate_of_turn/window_ms");
  ConfigItem(rate_of_turn_window_ms)
      ->set_title("Rate of Turn Window (ms)")
      ->set_description(
          "How far back heading samples are kept for the rate-of-turn fit. "
          "Longer = smoother but laggier; shorter = more responsive but "
          "noisier.")
      ->set_sort_order(59)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Window (ms)","type":"integer","minimum":500,"maximum":10000}}})schema");

  auto rate_of_turn_min_span_ms = std::make_shared<PersistingObservableValue<int>>(
      static_cast<int>(kRateOfTurnMinSpanMs), "/rate_of_turn/min_span_ms");
  ConfigItem(rate_of_turn_min_span_ms)
      ->set_title("Rate of Turn Minimum Span (ms)")
      ->set_description(
          "Minimum elapsed time between the oldest and newest sample in "
          "the window before a rate-of-turn value is produced at all -- "
          "below this, ordinary sensor noise over a very short span would "
          "read as wild rate swings. Clamped to never exceed the window "
          "above (a larger minimum span could never be satisfied).")
      ->set_sort_order(60)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Minimum span (ms)","type":"integer","minimum":100,"maximum":10000}}})schema");

  // --- NMEA 2000 (CAN bus via TWAI) ---

  // Identifies as a B&G Precision-9 (SPEC.md §1.2, §5.1, §10) — product
  // info and device function/class/manufacturer code are cloned from
  // htool/ESP32_Precision-9_compass_CMPS14. The "unique number" below is
  // deliberately NOT cloned; it's derived from this board's own MAC
  // (GetBoardSerialNumber()), per the design decision in SPEC.md §10.
  nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);
  nmea2000->SetN2kCANSendFrameBufSize(150);
  nmea2000->SetN2kCANReceiveFrameBufSize(150);
  nmea2000->SetProductInformation(
      kProductModelSerialCode,
      kProductCode,
      kProductModelId,
      kProductSoftwareVersion,
      kProductModelVersion
  );
  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),  // unique number — MAC-derived, not cloned
      kDeviceFunction,
      kDeviceClass,
      kManufacturerCode
  );
  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 74);
  nmea2000->EnableForward(false);
  nmea2000->ExtendTransmitMessages(kTransmitMessages);
  nmea2000->Open();

  // Process N2K messages (address claim, heartbeat, etc.)
  event_loop()->onRepeat(1, []() { nmea2000->ParseMessages(); });

  auto heading_sender = new halser::N2kHeadingSender(nmea2000);
  auto rate_of_turn_sender = new halser::N2kRateOfTurnSender(nmea2000);

  event_loop()->onRepeat(100, [heading_sender, rate_of_turn_sender, n2k_enabled,
                                n2k_heading_pgn_enabled,
                                n2k_rate_of_turn_pgn_enabled]() {
    if (n2k_enabled->get() && n2k_heading_pgn_enabled->get()) {
      heading_sender->send();
    }
    if (n2k_enabled->get() && n2k_rate_of_turn_pgn_enabled->get()) {
      rate_of_turn_sender->send();
    }
  });

  // --- SignalK output ---

  // Fault indication (SPEC.md §6) is meta.timeout, not an active
  // notification — see docs/plans/fault-indication.md for why. Any
  // SignalK-spec-aware consumer computes staleness itself from this
  // advisory value and the delta's own timestamp; this firmware doesn't
  // have to declare an alarm state at all.
  auto sk_heading_output = new SKOutputFloat(
      "navigation.headingMagnetic", "/signalk/heading_path",
      new SKMetadata("rad", "", "", "", kHeadingTimeoutSeconds));

  // navigation.rateOfTurn (rad/s, +ve = starboard) is a standard SignalK
  // key (see SPEC.md §5.1 for the equivalent N2K PGN 127251) that this
  // firmware previously computed but never published to SignalK.
  auto sk_rate_of_turn_output = new SKOutputFloat(
      "navigation.rateOfTurn", "/signalk/rate_of_turn_path",
      new SKMetadata("rad/s", "", "", "", kHeadingTimeoutSeconds));

  // navigation.headingTrue (SPEC.md §5.1a): magnetic heading + bus-
  // sourced variation, only set when a recent variation is available
  // (see the heading_producer consumer lambda below).
  auto sk_heading_true_output = new SKOutputFloat(
      "navigation.headingTrue", "/signalk/heading_true_path",
      new SKMetadata("rad", "", "", "", kVariationTimeoutSeconds));

  // Raw magnetic field (SPEC.md §5.2): custom sensors.* paths, no
  // established standard, no unit (the manual doesn't document a
  // counts-to-µT conversion factor) — raw sensor counts as-is, same
  // values already visible via the serial terminal (§8.1). description_
  // is filled in on all three since SignalK requires it for any
  // non-standard path (SKMetadata's own doc comment); display_name_/
  // short_name_ likewise, since consumers have nothing else to show.
  auto sk_mag_x_output = new SKOutputFloat(
      "sensors.hwt3100.magneticField.x", "/signalk/mag_x_path",
      new SKMetadata(
          "", "Mag X",
          "Raw, uncalibrated magnetic field X-axis reading from the "
          "HWT3100 compass module, in sensor counts -- the manual "
          "doesn't document a counts-to-µT conversion factor, so "
          "no unit is given. Diagnostic-only.",
          "MagX", kHeadingTimeoutSeconds));
  auto sk_mag_y_output = new SKOutputFloat(
      "sensors.hwt3100.magneticField.y", "/signalk/mag_y_path",
      new SKMetadata(
          "", "Mag Y",
          "Raw, uncalibrated magnetic field Y-axis reading from the "
          "HWT3100 compass module, in sensor counts -- the manual "
          "doesn't document a counts-to-µT conversion factor, so "
          "no unit is given. Diagnostic-only.",
          "MagY", kHeadingTimeoutSeconds));
  auto sk_mag_z_output = new SKOutputFloat(
      "sensors.hwt3100.magneticField.z", "/signalk/mag_z_path",
      new SKMetadata(
          "", "Mag Z",
          "Raw, uncalibrated magnetic field Z-axis reading from the "
          "HWT3100 compass module, in sensor counts -- the manual "
          "doesn't document a counts-to-µT conversion factor, so "
          "no unit is given. Diagnostic-only.",
          "MagZ", kHeadingTimeoutSeconds));

  // --- HWT3100 serial I/O, calibration offset, and dispatch to outputs ---

  auto heading_producer = new TaskQueueProducer<HeadingReading>(HeadingReading{});
  auto raw_line_producer =
      new TaskQueueProducer<HWT3100RawLine>(HWT3100RawLine{});

  auto serial_terminal = std::make_shared<halser::SerialTerminal>("/hwt3100/serial_log");
  ConfigItem(serial_terminal)
      ->set_title("HWT3100 Serial Log")
      ->set_description(
          "Read-only: the most recent raw lines received from the HWT3100 (SPEC.md §8.1)")
      ->set_sort_order(10)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"lines":{"title":"Lines","type":"array","items":{"type":"string"}}}})schema");

  raw_line_producer->connect_to(new LambdaConsumer<HWT3100RawLine>(
      [serial_terminal](HWT3100RawLine line) { serial_terminal->AddLine(line); }));

  // AT+PRATE=? reply handling (SPEC.md §8.2b): the module's "+PRATE=<n>"
  // response arrives on this same raw-line stream, like any other line
  // — there's no separate reply channel. Only accepted while
  // output_prate is still kPrateUnknown, so a stray/unexpected
  // "+PRATE=" line can never silently override an already-known
  // (learned or user-configured) value.
  raw_line_producer->connect_to(new LambdaConsumer<HWT3100RawLine>(
      [output_prate](HWT3100RawLine line) {
        if (output_prate->get() != halser::kPrateUnknown) return;
        int learned = 0;
        if (halser::ParsePrateReply(line.text, &learned)) {
          output_prate->set(learned);
        }
      }));

  // Calibration command reply handling (SPEC.md §8.2): shows the
  // module's own plain-text reply ("Calibrating" etc.) on the Status
  // page after a Control-tab button fires the corresponding AT+CALI
  // command, so the user gets some indication of what actually
  // happened rather than a purely fire-and-forget button (§10 Design
  // Decisions — full synchronous request/response isn't available
  // without further upstream SensESP changes; see
  // docs/plans/calibration-control-tab.md). Like the AT+PRATE reply
  // above, this rides the same raw-line stream as ordinary heading
  // data — IsCalibrationReply() picks out only the three known reply
  // strings.
  auto calibration_status = std::make_shared<StatusPageItem<String>>(
      "HWT3100 Calibration Reply", "(none yet)", "HWT3100", 30);
  raw_line_producer->connect_to(new LambdaConsumer<HWT3100RawLine>(
      [calibration_status](HWT3100RawLine line) {
        if (halser::IsCalibrationReply(line.text)) {
          calibration_status->set(String(line.text));
        }
      }));

  auto rate_of_turn_estimator = new halser::RateOfTurnEstimator(
      kRateOfTurnWindowMs, kRateOfTurnMinSpanMs);

  // Applies both persisted values to the estimator, clamping min_span
  // to never exceed window (SPEC.md §7) — a larger min_span could
  // never be satisfied, since the window itself caps the maximum
  // possible sample span.
  auto apply_rate_of_turn_config = [rate_of_turn_estimator, rate_of_turn_window_ms,
                                     rate_of_turn_min_span_ms]() {
    unsigned long window = static_cast<unsigned long>(rate_of_turn_window_ms->get());
    unsigned long min_span =
        static_cast<unsigned long>(rate_of_turn_min_span_ms->get());
    if (min_span > window) min_span = window;
    rate_of_turn_estimator->SetWindowMs(window);
    rate_of_turn_estimator->SetMinSpanMs(min_span);
  };
  apply_rate_of_turn_config();
  rate_of_turn_window_ms->connect_to(
      new LambdaConsumer<int>([apply_rate_of_turn_config](int) {
        apply_rate_of_turn_config();
      }));
  rate_of_turn_min_span_ms->connect_to(
      new LambdaConsumer<int>([apply_rate_of_turn_config](int) {
        apply_rate_of_turn_config();
      }));

  heading_producer->connect_to(new LambdaConsumer<HeadingReading>(
      [=](HeadingReading reading) {
        HeadingReading corrected =
            halser::ApplyCalibrationOffset(reading, heading_offset->get());
        // HeadingReading.heading is degrees throughout this firmware's
        // internal pipeline (matches the HWT3100's own wire format and
        // RateOfTurnEstimator's wraparound math) — both N2K's
        // SetN2kPGN127250 and SignalK's navigation.headingMagnetic
        // require radians, so the conversion happens right at each
        // output boundary, not upstream.
        float heading_rad = corrected.heading * kDegreesToRadians;
        heading_sender->heading_.update(heading_rad);
        if (signalk_enabled->get() && sk_heading_enabled->get()) {
          sk_heading_output->set(heading_rad);
        }

        // navigation.headingTrue (SPEC.md §5.1a): only computed/published
        // when a recent bus-sourced variation exists — omitted, not sent
        // as a placeholder, when it doesn't (same "don't fabricate data"
        // pattern as rate of turn before enough history exists).
        if (heading_sender->variation_.is_valid()) {
          float heading_true_rad = fmodf(
              heading_rad + heading_sender->variation_.value(), kTwoPi);
          if (heading_true_rad < 0.0f) heading_true_rad += kTwoPi;
          if (signalk_enabled->get() && sk_heading_true_enabled->get()) {
            sk_heading_true_output->set(heading_true_rad);
          }
        }

        rate_of_turn_estimator->AddSample(corrected.heading, corrected.timestamp);
        float rate_of_turn = 0.0f;
        if (rate_of_turn_estimator->GetRateOfTurn(&rate_of_turn)) {
          rate_of_turn_sender->rate_of_turn_.update(rate_of_turn);
          if (signalk_enabled->get() && sk_rate_of_turn_enabled->get()) {
            sk_rate_of_turn_output->set(rate_of_turn);
          }
        }

        if (signalk_enabled->get() && raw_mag_field_enabled->get()) {
          sk_mag_x_output->set(static_cast<float>(corrected.mag_x));
          sk_mag_y_output->set(static_cast<float>(corrected.mag_y));
          sk_mag_z_output->set(static_cast<float>(corrected.mag_z));
        }
      }));

  auto hwt3100_serial =
      new halser::HWT3100SerialIO(Serial1, heading_producer, raw_line_producer);

  // Baud auto-detection (SPEC.md §8.2c): if the persisted baud is still
  // unknown, try each candidate in turn (passive listening only, no
  // AT+UART sent) and persist whichever one produces valid data. Must
  // happen before Begin() starts the background read task, since
  // DetectBaud() isn't safe to call concurrently with it. If nothing is
  // found in this pass, don't persist a guess -- read at the
  // recommended default for this boot and let the next boot retry
  // detection fresh.
  int startup_baud = kHWT3100DefaultBaud;
  if (hwt3100_baud->get() == halser::kBaudUnknown) {
    int detected = 0;
    if (hwt3100_serial->DetectBaud(kBaudCandidates, kNumBaudCandidates,
                                    kBaudDetectTimeoutMs, kUART1RxPin,
                                    kUART1TxPin, &detected)) {
      hwt3100_baud->set(detected);
      startup_baud = detected;
    }
  } else {
    startup_baud = hwt3100_baud->get();
  }
  hwt3100_serial->Begin(startup_baud, kUART1RxPin, kUART1TxPin);

  // Runtime baud switching: only fires on an explicit config change
  // *after* the wiring above, since it's attached after any startup
  // hwt3100_baud->set() from auto-detection -- so discovering the
  // module's existing rate never itself triggers an unwanted AT+UART
  // command.
  hwt3100_baud->connect_to(new LambdaConsumer<int>([hwt3100_serial](int value) {
    if (value != halser::kBaudUnknown) {
      hwt3100_serial->SetBaudRate(value, kUART1RxPin, kUART1TxPin);
    }
  }));

  // Re-apply the persisted AT+FILT setting on every boot (the module
  // can't report its current filter back to us) and again whenever the
  // config value changes.
  hwt3100_serial->SetOutputFilter(output_filter->get());
  output_filter->connect_to(new LambdaConsumer<int>(
      [hwt3100_serial](int value) { hwt3100_serial->SetOutputFilter(value); }));

  // AT+PRATE (SPEC.md §8.2b): if the persisted rate is still unknown,
  // query the module instead of guessing — forcing an arbitrary default
  // here could be actively harmful (AT+PRATE=0 silences the continuous
  // stream this firmware's read pipeline depends on). The reply is
  // picked up by the raw-line consumer above; once parsed, it's
  // persisted via output_prate->set(), which also re-applies it through
  // the connect_to() below (a harmless echo of what the module just
  // told us). If the rate is already known (learned on a previous boot,
  // or set explicitly via config), (re-)apply it directly instead.
  if (output_prate->get() == halser::kPrateUnknown) {
    hwt3100_serial->QueryOutputRate();
  } else {
    hwt3100_serial->SetOutputRate(output_prate->get());
  }
  output_prate->connect_to(new LambdaConsumer<int>([hwt3100_serial](int value) {
    if (value != halser::kPrateUnknown) {
      hwt3100_serial->SetOutputRate(value);
    }
  }));

  // --- Calibration commands (SPEC.md §8.2) ---

  auto calibration_commands =
      new halser::CalibrationCommandHandler(hwt3100_serial);

  // Lets a compatible MFD start/stop calibration over the N2K bus, the
  // same way htool/ESP32_Precision-9_compass_CMPS14 does (SPEC.md §8.2,
  // §10) — reverse-engineered proprietary protocol, unverified against
  // real hardware; see docs/plans/mfd-calibration.md. Self-attaches to
  // nmea2000 via its tMsgHandler base constructor.
  new halser::MfdCalibrationBridge(nmea2000, calibration_commands);

  // Listens for PGN 127258 (Magnetic Variation) from another N2K
  // device (SPEC.md §5.1a) — feeds heading_sender's variation_ so PGN
  // 127250's own Variation field and navigation.headingTrue (below)
  // both get real data when a source exists on the bus. Read-only:
  // never transmits PGN 127258 itself. Self-attaches to nmea2000 via
  // its tMsgHandler base constructor, same as MfdCalibrationBridge.
  new halser::MagneticVariationListener(nmea2000, &heading_sender->variation_);

  // Three adjacent, consistently-worded actions (SPEC.md §8.2), now
  // real UIButtons on the web UI's Control tab (BoatHacks/SensESP;
  // see the platformio.ini comment and docs/plans/calibration-control-tab.md
  // — temporary until the upstream PR lands). must_confirm is left at
  // its default (true) for all three: each one changes the module's
  // on-module magnetic calibration state, which is annoying to redo if
  // clicked by accident. Fire-and-forget by design — UIButton has no
  // return-value mechanism — so calibration_status (above) is what
  // shows whether/what the module actually replied.
  //
  // The Control tab renders buttons in UIButton::get_ui_buttons()'s
  // std::map key order, i.e. sorted by the `name` argument below, not
  // registration order — the "1_"/"2_"/"3_" prefixes are what actually
  // pin the displayed order to Start, Stop, Clear.
  //
  // The Start button's title carries the calibration procedure itself
  // (rotate, then Stop) — UIButton has no separate description field
  // (see docs/plans/calibration-control-tab.md), and the title is the
  // only per-button text the Control tab currently renders, so this is
  // the only channel available for it without another round of
  // BoatHacks/SensESP changes.
  sensesp::UIButton::add(
      "hwt3100_calibration_1_start",
      "Start Calibration (rotate 360° at least 3x, then press Stop)")
      ->attach([calibration_commands]() { calibration_commands->StartCalibration(); });

  sensesp::UIButton::add("hwt3100_calibration_2_stop", "Stop Calibration")
      ->attach([calibration_commands]() { calibration_commands->EndCalibration(); });

  sensesp::UIButton::add("hwt3100_calibration_3_clear", "Clear Calibration")
      ->attach([calibration_commands]() { calibration_commands->ClearCalibration(); });

  while (true) {
    event_loop()->tick();
  }
}
