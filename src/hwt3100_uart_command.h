#ifndef HALSER_SRC_HWT3100_UART_COMMAND_H_
#define HALSER_SRC_HWT3100_UART_COMMAND_H_

#include <cstddef>

namespace halser {

// Sentinel meaning "not yet known" for the persisted HWT3100 UART baud
// rate (gateway.cpp) — never sent to the module or passed to
// Serial1.begin(). See SPEC.md §8.2c.
constexpr int kBaudUnknown = -1;

// The three baud rates AT+UART actually supports (manual's AT-command
// table: AT+UART=0/1/2 -> 9600/115200/460800).
constexpr int kBaud9600 = 9600;
constexpr int kBaud115200 = 115200;
constexpr int kBaud460800 = 460800;

// Formats an AT+UART command selecting one of the three supported baud
// rates. `requested_baud` is snapped to the nearest of {9600, 115200,
// 460800} (ties broken toward the lower rate) rather than rejected —
// there's no way to signal an error back through this pure function's
// interface, and clamping to the nearest valid rate is a safer failure
// mode than sending a malformed command. Returns the baud rate actually
// selected (and written into the command as the matching AT+UART
// index), so the caller knows what to reconfigure Serial1 to.
int FormatUartCommand(int requested_baud, char* buf, size_t buf_len);

}  // namespace halser

#endif  // HALSER_SRC_HWT3100_UART_COMMAND_H_
