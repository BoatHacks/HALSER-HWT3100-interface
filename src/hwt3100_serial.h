#ifndef HALSER_SRC_HWT3100_SERIAL_H_
#define HALSER_SRC_HWT3100_SERIAL_H_

#include <HardwareSerial.h>

#include "hwt3100_types.h"
#include "sensesp/system/task_queue_producer.h"

namespace halser {

// Owns the HWT3100's UART link exclusively (SPEC.md §2, ARCHITECTURE.md
// §2.1, §6). No other component in this firmware should hold a
// reference to the underlying HardwareSerial.
//
// Runs a dedicated FreeRTOS task that reads the module's ASCII output,
// parses it via ParseHWT3100Line() (hwt3100_parser.h), and marshals
// results to the main SensESP loop. Also exposes the *only* write paths
// to the module: SendCommand() (three closed HWT3100Command values),
// SetOutputFilter() (AT+FILT with a bounds-clamped integer),
// SetOutputRate()/QueryOutputRate() (AT+PRATE, same clamping approach),
// and SetBaudRate() (AT+UART, snapped to the nearest of three supported
// rates — SPEC.md §8.2a/§8.2b/§8.2c, §9). There is deliberately no
// method here that accepts raw bytes or arbitrary text. DetectBaud() is
// the one method here that writes nothing at all — pure passive
// listening, used only before Begin() starts the background task.
class HWT3100SerialIO {
 public:
  // heading_producer/raw_line_producer must outlive this object and the
  // FreeRTOS task started by Begin(). They're owned by the caller
  // (gateway.cpp) because constructing a TaskQueueProducer requires the
  // main event loop, which this class has no business knowing about.
  HWT3100SerialIO(HardwareSerial& serial,
                   sensesp::TaskQueueProducer<HeadingReading>* heading_producer,
                   sensesp::TaskQueueProducer<HWT3100RawLine>* raw_line_producer);

  // Starts the serial port and the dedicated read task. Call once, from
  // the main setup path.
  void Begin(unsigned long baud, int rx_pin, int tx_pin);

  // Transmits one of HWT3100Command's three known-safe values — no
  // overload, no debug backdoor (ARCHITECTURE.md §6).
  void SendCommand(HWT3100Command cmd);

  // Sends AT+FILT=<value>, clamped to [0, 999] by FormatFilterCommand()
  // (hwt3100_filter_command.h) before anything reaches the wire — the
  // one parameterized write this class allows, still not a raw-text
  // backdoor (SPEC.md §8.2/§9, ARCHITECTURE.md §6).
  void SetOutputFilter(int value);

  // Sends AT+PRATE=<value>, clamped by FormatPrateCommand()
  // (hwt3100_prate_command.h) to {0} u [10, 10000] before anything
  // reaches the wire (SPEC.md §8.2b).
  void SetOutputRate(int value);

  // Sends the fixed query "AT+PRATE=?\r\n" to ask the module for its
  // current output rate (SPEC.md §8.2b). The module's "+PRATE=<n>\r\n"
  // reply arrives like any other line on raw_line_producer — there is
  // no separate reply channel — and is parsed by the caller
  // (gateway.cpp) via ParsePrateReply() (hwt3100_prate_command.h).
  void QueryOutputRate();

  // Synchronous, blocking baud-rate discovery (SPEC.md §8.2c). Tries
  // each of candidate_bauds[0..num_candidates) in order: opens the
  // port at that rate and listens up to per_baud_timeout_ms for at
  // least one line that parses as valid HWT3100 output. Returns true
  // and fills *detected_baud on the first candidate that works, false
  // if none do. Sends no AT commands — this is passive listening, not
  // reconfiguration. Must be called before Begin() starts the
  // background read task (no concurrent access to serial_ yet at that
  // point), never after.
  bool DetectBaud(const int* candidate_bauds, size_t num_candidates,
                   unsigned long per_baud_timeout_ms, int rx_pin, int tx_pin,
                   int* detected_baud);

  // Sends AT+UART=<n> (via FormatUartCommand(), hwt3100_uart_command.h,
  // which snaps requested_baud to the nearest of the three supported
  // rates) at whatever rate the port is currently open at, waits a
  // fixed settle delay, then reconfigures the port to the new rate.
  // Returns the baud rate actually selected/switched to. See SPEC.md
  // §8.2c and §11 for the unverified timing assumptions this makes.
  int SetBaudRate(int requested_baud, int rx_pin, int tx_pin);

 private:
  static void ReadTaskTrampoline(void* arg);
  void ReadTaskLoop();

  HardwareSerial& serial_;
  sensesp::TaskQueueProducer<HeadingReading>* heading_producer_;
  sensesp::TaskQueueProducer<HWT3100RawLine>* raw_line_producer_;

  char line_buffer_[HWT3100RawLine::kMaxLength];
  size_t line_length_ = 0;
};

}  // namespace halser

#endif  // HALSER_SRC_HWT3100_SERIAL_H_
