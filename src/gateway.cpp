#include "gateway.h"

#include <Adafruit_NeoPixel.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <esp_mac.h>

#include "calibration_offset.h"
#include "halser_const.h"
#include "hwt3100_calibration_commands.h"
#include "hwt3100_serial.h"
#include "hwt3100_types.h"
#include "n2k_senders.h"
#include "rate_of_turn.h"
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
Adafruit_NeoPixel* led = nullptr;

// Rate-of-turn sliding window (SPEC.md §11 flags these as reasonable
// defaults needing real-hardware tuning, not values derived from an
// actual helm/autopilot's sensitivity requirements).
constexpr unsigned long kRateOfTurnWindowMs = 2000;
constexpr unsigned long kRateOfTurnMinSpanMs = 500;

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

  // RGB LED. Fault indication (SPEC.md §6) is not implemented yet — see
  // docs/plans/gateway-wiring.md; this is initialization only for now.
  led = new Adafruit_NeoPixel(1, kRGBLEDPin, NEO_GRB + NEO_KHZ800);
  led->begin();
  led->setBrightness(30);

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

  auto heading_offset = std::make_shared<PersistingObservableValue<float>>(
      0.0f, "/calibration/heading_offset");
  ConfigItem(heading_offset)
      ->set_title("Heading Calibration Offset")
      ->set_description(
          "Degrees added to the raw HWT3100 heading to correct for mounting misalignment")
      ->set_sort_order(50)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Offset (degrees)","type":"number"}}})schema");

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

  auto sk_heading_output =
      new SKOutputFloat("navigation.headingMagnetic", "/signalk/heading_path");

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

  auto rate_of_turn_estimator = new halser::RateOfTurnEstimator(
      kRateOfTurnWindowMs, kRateOfTurnMinSpanMs);

  heading_producer->connect_to(new LambdaConsumer<HeadingReading>(
      [=](HeadingReading reading) {
        HeadingReading corrected =
            halser::ApplyCalibrationOffset(reading, heading_offset->get());
        heading_sender->heading_.update(corrected.heading);
        if (signalk_enabled->get()) {
          sk_heading_output->set(corrected.heading);
        }

        rate_of_turn_estimator->AddSample(corrected.heading, corrected.timestamp);
        float rate_of_turn = 0.0f;
        if (rate_of_turn_estimator->GetRateOfTurn(&rate_of_turn)) {
          rate_of_turn_sender->rate_of_turn_.update(rate_of_turn);
        }
      }));

  auto hwt3100_serial =
      new halser::HWT3100SerialIO(Serial1, heading_producer, raw_line_producer);
  hwt3100_serial->Begin(kHWT3100DefaultBaud, kUART1RxPin, kUART1TxPin);

  // --- Calibration commands (SPEC.md §8.2) ---

  auto calibration_commands =
      new halser::CalibrationCommandHandler(hwt3100_serial);

  auto start_calibration = std::make_shared<PersistingObservableValue<bool>>(
      false, "/hwt3100/calibration/start");
  ConfigItem(start_calibration)
      ->set_title("Start Magnetic Calibration")
      ->set_description(
          "Set true to send AT+CALI=1. Rotate the module 2-3 full turns after starting.")
      ->set_sort_order(20)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Start","type":"boolean"}}})schema");
  WireCalibrationTrigger(start_calibration,
                         [calibration_commands]() { calibration_commands->StartCalibration(); });

  auto end_calibration = std::make_shared<PersistingObservableValue<bool>>(
      false, "/hwt3100/calibration/end");
  ConfigItem(end_calibration)
      ->set_title("End Magnetic Calibration")
      ->set_description("Set true to send AT+CALI=0.")
      ->set_sort_order(21)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"End","type":"boolean"}}})schema");
  WireCalibrationTrigger(end_calibration,
                         [calibration_commands]() { calibration_commands->EndCalibration(); });

  auto clear_calibration = std::make_shared<PersistingObservableValue<bool>>(
      false, "/hwt3100/calibration/clear");
  ConfigItem(clear_calibration)
      ->set_title("Clear Magnetic Calibration")
      ->set_description("Set true to send AT+CALI=2 (resets the module's magnetic offset).")
      ->set_sort_order(22)
      ->set_config_schema(
          R"schema({"type":"object","properties":{"value":{"title":"Clear","type":"boolean"}}})schema");
  WireCalibrationTrigger(clear_calibration,
                         [calibration_commands]() { calibration_commands->ClearCalibration(); });

  while (true) {
    event_loop()->tick();
  }
}
