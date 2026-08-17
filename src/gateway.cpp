#include "gateway.h"

#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <esp_mac.h>

#include "calibration_offset.h"
#include "halser_const.h"
#include "hwt3100_calibration_commands.h"
#include "hwt3100_prate_command.h"
#include "hwt3100_serial.h"
#include "hwt3100_types.h"
#include "hwt3100_uart_command.h"
#include "mfd_calibration_bridge.h"
#include "n2k_senders.h"
#include "rate_of_turn.h"
#include "sensesp/signalk/signalk_metadata.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/observablevalue.h"
#include "sensesp/system/task_queue_producer.h"
#include "sensesp/ui/config_item.h"
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

/// Wires a PersistingObservableValue<bool> up as a one-shot "trigger":
/// setting it true fires `action`, then it resets itself to false so it
/// can be triggered again. See docs/plans/gateway-wiring.md for the
/// reboot-replay trade-off this pattern accepts.
void WireCalibrationTrigger(std::shared_ptr<PersistingObservableValue<bool>> trigger,
                             std::function<void()> action) {
  trigger->connect_to(new LambdaConsumer<bool>([trigger, action](bool value) {
    if (value) {
      action();
      trigger->set(false);
    }
  }));
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
      ->set_sort_order(121)
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

  // Raw magnetic field (SPEC.md §5.2): custom sensors.* paths, no
  // established standard, no unit (the manual doesn't document a
  // counts-to-µT conversion factor) — raw sensor counts as-is, same
  // values already visible via the serial terminal (§8.1).
  auto sk_mag_x_output = new SKOutputFloat(
      "sensors.hwt3100.magneticField.x", "/signalk/mag_x_path",
      new SKMetadata("", "", "", "", kHeadingTimeoutSeconds));
  auto sk_mag_y_output = new SKOutputFloat(
      "sensors.hwt3100.magneticField.y", "/signalk/mag_y_path",
      new SKMetadata("", "", "", "", kHeadingTimeoutSeconds));
  auto sk_mag_z_output = new SKOutputFloat(
      "sensors.hwt3100.magneticField.z", "/signalk/mag_z_path",
      new SKMetadata("", "", "", "", kHeadingTimeoutSeconds));

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

  auto rate_of_turn_estimator = new halser::RateOfTurnEstimator(
      kRateOfTurnWindowMs, kRateOfTurnMinSpanMs);

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
        if (signalk_enabled->get()) {
          sk_heading_output->set(heading_rad);
        }

        rate_of_turn_estimator->AddSample(corrected.heading, corrected.timestamp);
        float rate_of_turn = 0.0f;
        if (rate_of_turn_estimator->GetRateOfTurn(&rate_of_turn)) {
          rate_of_turn_sender->rate_of_turn_.update(rate_of_turn);
          if (signalk_enabled->get()) {
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

  // Three adjacent, consistently-worded actions (SPEC.md §8.2): each is
  // a one-shot trigger, not a persistent setting — SensESP's config UI
  // has no "button" primitive, so the mechanism is check-the-box, save,
  // and it flips back to unchecked once the command fires (§10 Design
  // Decisions covers the reboot-replay trade-off this accepts). The
  // "1/2/3" title prefix and matching sort_order keep the three
  // together and in the order you'd actually use them, and every
  // description spells out the same "check + save, then it un-checks
  // itself" mechanic so it reads the same way three times in a row
  // rather than leaving it to be inferred from just the first one.
  auto start_calibration = std::make_shared<PersistingObservableValue<bool>>(
      false, "/hwt3100/calibration/start");
  ConfigItem(start_calibration)
      ->set_title("Calibration 1/3: Start")
      ->set_description(
          "One-shot action, not a setting: check the box and Save to send "
          "AT+CALI=1 and begin calibration; it un-checks itself once sent. "
          "Rotate the module through 2-3 full turns after starting.")
      ->set_sort_order(20)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Send AT+CALI=1","type":"boolean"}}})schema");
  WireCalibrationTrigger(start_calibration,
                         [calibration_commands]() { calibration_commands->StartCalibration(); });

  auto end_calibration = std::make_shared<PersistingObservableValue<bool>>(
      false, "/hwt3100/calibration/end");
  ConfigItem(end_calibration)
      ->set_title("Calibration 2/3: End")
      ->set_description(
          "One-shot action, not a setting: check the box and Save to send "
          "AT+CALI=0 and finish calibration; it un-checks itself once sent.")
      ->set_sort_order(21)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Send AT+CALI=0","type":"boolean"}}})schema");
  WireCalibrationTrigger(end_calibration,
                         [calibration_commands]() { calibration_commands->EndCalibration(); });

  auto clear_calibration = std::make_shared<PersistingObservableValue<bool>>(
      false, "/hwt3100/calibration/clear");
  ConfigItem(clear_calibration)
      ->set_title("Calibration 3/3: Clear")
      ->set_description(
          "One-shot action, not a setting: check the box and Save to send "
          "AT+CALI=2 and reset the module's magnetic offset; it un-checks "
          "itself once sent.")
      ->set_sort_order(22)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Send AT+CALI=2","type":"boolean"}}})schema");
  WireCalibrationTrigger(clear_calibration,
                         [calibration_commands]() { calibration_commands->ClearCalibration(); });

  while (true) {
    event_loop()->tick();
  }
}
