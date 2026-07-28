#include "link/serial_link.h"
#include "elm327/line_buffer.h"
#include "link/write_all.h"
#include <Arduino.h>

namespace {

void (*g_handler)(const char*, std::string&) = nullptr;
LineBuffer g_buffer;

} // namespace

void serialLinkSetLineHandler(void (*handler)(const char* line, std::string& reply)) {
    g_handler = handler;
}

void serialLinkPoll() {
    if (!g_handler) return;

    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (!g_buffer.offer(c)) continue;

        std::string reply;
        if (g_buffer.overflowed()) {
            // The command was discarded, so it must not be executed. Answer as
            // the adapter answers any unparseable input.
            reply = "?\r>";
        } else {
            g_handler(g_buffer.line(), reply);
        }
        // USBCDC::write returns a SHORT COUNT when the TX FIFO is full and its
        // internal timeout expires — it does not promise to send the whole
        // buffer. Ignoring the return value dropped the tail of a reply,
        // usually including the ">" prompt the host waits for, so the client
        // saw a reply that never ended. Under sustained scanning this stopped
        // whole runs. See link/write_all.h.
        writeAll([](const char* data, size_t size) {
                     return Serial.write(data, size);
                 },
                 reply.data(), reply.size());
        Serial.flush();
    }
}
