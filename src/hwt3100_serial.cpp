#include "hwt3100_serial.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdio>

#include "hwt3100_filter_command.h"
#include "hwt3100_parser.h"
#include "hwt3100_prate_command.h"
#include "hwt3100_uart_command.h"

namespace halser {

namespace {

// AT command text for each HWT3100Command value (SPEC.md §8.2). All
// terminated \r\n per the HWT3100-TTL/232 manual ("All AT commands end
// with a carriage return and line feed"). This table is the entire
// write surface of this firmware to the HWT3100 — there is deliberately
// no entry, and no possible path to construct one, for AT+MODE.
const char* CommandText(HWT3100Command cmd) {
  switch (cmd) {
    case HWT3100Command::kStartCalibration:
      return "AT+CALI=1\r\n";
    case HWT3100Command::kEndCalibration:
      return "AT+CALI=0\r\n";
    case HWT3100Command::kClearCalibration:
      return "AT+CALI=2\r\n";
  }
  // No default: case. HWT3100Command is a closed, exhaustive enum
  // (ARCHITECTURE.md §6) — every value must map to a known-safe command
  // here, so a new enum value with no corresponding case is a compiler
  // warning (-Wswitch), not a silent gap that falls through to sending
  // nothing or, worse, something unintended.
  return nullptr;
}

constexpr int kReadTaskStackSize = 4096;
constexpr UBaseType_t kReadTaskPriority = 1;
constexpr TickType_t kReadTaskPollDelay = pdMS_TO_TICKS(5);

}  // namespace

HWT3100SerialIO::HWT3100SerialIO(
    HardwareSerial& serial,
    sensesp::TaskQueueProducer<HeadingReading>* heading_producer,
    sensesp::TaskQueueProducer<HWT3100RawLine>* raw_line_producer)
    : serial_(serial),
      heading_producer_(heading_producer),
      raw_line_producer_(raw_line_producer) {}

void HWT3100SerialIO::Begin(unsigned long baud, int rx_pin, int tx_pin) {
  serial_.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
  xTaskCreate(&HWT3100SerialIO::ReadTaskTrampoline, "hwt3100_read",
              kReadTaskStackSize, this, kReadTaskPriority, nullptr);
}

void HWT3100SerialIO::SendCommand(HWT3100Command cmd) {
  const char* text = CommandText(cmd);
  if (text != nullptr) {
    serial_.print(text);
  }
}

void HWT3100SerialIO::SetOutputFilter(int value) {
  char buf[24];
  FormatFilterCommand(value, buf, sizeof(buf));
  serial_.print(buf);
}

void HWT3100SerialIO::SetOutputRate(int value) {
  char buf[24];
  FormatPrateCommand(value, buf, sizeof(buf));
  serial_.print(buf);
}

void HWT3100SerialIO::QueryOutputRate() { serial_.print("AT+PRATE=?\r\n"); }

bool HWT3100SerialIO::DetectBaud(const int* candidate_bauds,
                                  size_t num_candidates,
                                  unsigned long per_baud_timeout_ms, int rx_pin,
                                  int tx_pin, int* detected_baud) {
  for (size_t i = 0; i < num_candidates; i++) {
    int baud = candidate_bauds[i];
    serial_.begin(baud, SERIAL_8N1, rx_pin, tx_pin);

    char buf[HWT3100RawLine::kMaxLength];
    size_t len = 0;
    bool found = false;
    unsigned long deadline = millis() + per_baud_timeout_ms;

    while (!found && millis() < deadline) {
      while (serial_.available()) {
        char c = static_cast<char>(serial_.read());

        if (c == '\r') continue;

        if (c == '\n') {
          buf[len] = '\0';
          HeadingReading reading;
          if (ParseHWT3100Line(buf, &reading)) {
            found = true;
            break;
          }
          len = 0;
          continue;
        }

        if (len < sizeof(buf) - 1) {
          buf[len++] = c;
        } else {
          len = 0;
        }
      }
      if (!found) {
        vTaskDelay(pdMS_TO_TICKS(5));
      }
    }

    serial_.end();

    if (found) {
      *detected_baud = baud;
      return true;
    }
  }
  return false;
}

int HWT3100SerialIO::SetBaudRate(int requested_baud, int rx_pin, int tx_pin) {
  // Settle delay between sending AT+UART and reconfiguring our own
  // side, chosen as a reasonable fixed value (SPEC.md §11 — the manual
  // doesn't document a timing spec for this transition).
  constexpr unsigned long kBaudSwitchSettleMs = 200;

  char buf[24];
  int actual_baud = FormatUartCommand(requested_baud, buf, sizeof(buf));
  serial_.print(buf);
  delay(kBaudSwitchSettleMs);
  serial_.begin(actual_baud, SERIAL_8N1, rx_pin, tx_pin);
  return actual_baud;
}

void HWT3100SerialIO::ReadTaskTrampoline(void* arg) {
  static_cast<HWT3100SerialIO*>(arg)->ReadTaskLoop();
}

void HWT3100SerialIO::ReadTaskLoop() {
  for (;;) {
    while (serial_.available()) {
      char c = static_cast<char>(serial_.read());

      if (c == '\r') {
        // Stripped rather than accumulated — ParseHWT3100Line() tolerates
        // a trailing \r either way, but not accumulating it keeps
        // line_buffer_ a plain "the fields" string, which is simpler to
        // reason about and to forward to the serial terminal (§8.1).
        continue;
      }

      if (c == '\n') {
        line_buffer_[line_length_] = '\0';

        HeadingReading reading;
        if (ParseHWT3100Line(line_buffer_, &reading)) {
          reading.timestamp = millis();
          if (heading_producer_ != nullptr) {
            heading_producer_->set(reading);
          }
        }

        if (raw_line_producer_ != nullptr) {
          HWT3100RawLine raw;
          snprintf(raw.text, sizeof(raw.text), "%s", line_buffer_);
          raw_line_producer_->set(raw);
        }

        line_length_ = 0;
        continue;
      }

      if (line_length_ < sizeof(line_buffer_) - 1) {
        line_buffer_[line_length_++] = c;
      } else {
        // Oversized line (garbled stream, wrong baud rate, ...) — drop
        // it rather than silently truncating into something that might
        // parse as different, wrong data.
        line_length_ = 0;
      }
    }
    vTaskDelay(kReadTaskPollDelay);
  }
}

}  // namespace halser
