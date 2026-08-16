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
// results to the main SensESP loop. Also exposes the *only* write path
// to the module: SendCommand(), which accepts nothing but the three
// closed HWT3100Command values — there is deliberately no method here
// that accepts raw bytes or text (SPEC.md §8.2).
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

  // The ONLY method that writes to the HWT3100. There is no overload, no
  // debug backdoor, and no way to make this method transmit anything
  // other than one of HWT3100Command's three known-safe values — see
  // ARCHITECTURE.md §6 for why that's the point.
  void SendCommand(HWT3100Command cmd);

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
