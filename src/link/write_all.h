#pragma once

#include <cstddef>

// A complete write over a sink that is allowed to accept less than it is given.
//
// PURE LOGIC, deliberately: link/*.cpp is excluded from env:native because it
// pulls in the Arduino BLE stack, so this lives in a header and takes its sink
// as a template parameter. That is what lets the retry contract be tested on
// the host rather than inferred from a board that only misbehaves under load.
//
// The bug this exists for: Arduino's USBCDC::write on the ESP32-S3 returns a
// SHORT COUNT when the TX FIFO is full and its internal timeout expires. It
// does not promise to send the whole buffer. serial_link.cpp ignored the
// return value, so under sustained scanning the tail of a reply -- often
// including the ">" prompt that tells the host the reply ended -- was silently
// dropped. The host then waited for a prompt that would never arrive and
// reported a timeout, which ended whole scan runs. Observed twice in ~26,000
// probes as `got b''` and `got b'NRC 3'`.

// How many consecutive zero-byte writes mean the sink is not coming back.
//
// Bounded on purpose. serialLinkPoll() runs inside loop(), so an unbounded
// retry against a host that has stopped draining CDC -- unplugged, or holding
// the port open without reading -- would wedge the firmware entirely. A gapped
// reply leaves the client to time out and retry, which is recoverable; a hung
// loop() is not. The count is generous enough that a merely busy FIFO drains
// long before it is reached.
constexpr int kWriteAllMaxStalledAttempts = 64;

// Writes `size` bytes from `data` through `write`, continuing across short
// writes. Returns the number of bytes actually written: `size` on success, or
// fewer if the sink stopped accepting bytes altogether.
//
// `write` must have the shape `size_t(const char*, size_t)` and return how
// many bytes it accepted, which may be zero.
//
// Taken by forwarding reference, NOT by value: a sink is usually a stateful
// object, and copying it would leave the caller's own state untouched while
// this function happily wrote through a duplicate.
template <typename WriteFn>
size_t writeAll(WriteFn&& write, const char* data, size_t size) {
    size_t sent = 0;
    int stalled = 0;
    while (sent < size) {
        const size_t n = write(data + sent, size - sent);
        if (n == 0) {
            if (++stalled >= kWriteAllMaxStalledAttempts) break;
            continue;
        }
        // Any progress at all clears the stall count: a FIFO that accepted
        // three bytes is draining, however slowly, and must not be abandoned
        // because of zero-writes that came before.
        stalled = 0;
        sent += n;
    }
    return sent;
}
