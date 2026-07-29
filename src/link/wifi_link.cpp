#include "link/wifi_link.h"
#include "elm327/line_buffer.h"
#include "link/write_all.h"
#include <WiFi.h>

namespace {

// The de-facto ELM327-over-WiFi port. Clones that use anything else are the
// exception, and every desktop tool defaults to this.
constexpr uint16_t kPort = 35000;

void (*g_handler)(const char*, std::string&) = nullptr;
WiFiServer g_server(kPort);
WiFiClient g_client;
LineBuffer g_buffer;

} // namespace

void wifiLinkSetLineHandler(void (*handler)(const char* line, std::string& reply)) {
    g_handler = handler;
}

void wifiLinkBegin(const char* ssid, const char* password) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    g_server.begin();
    // ELM327 replies are a few dozen bytes and the client will not send its
    // next command until the prompt arrives, so Nagle has nothing to coalesce
    // and only adds its delay to every single command.
    g_server.setNoDelay(true);
}

void wifiLinkPoll() {
    if (!g_handler) return;

    if (g_client && !g_client.connected()) {
        g_client.stop();
        g_buffer.reset();
    }

    if (WiFiClient incoming = g_server.accept()) {
        if (g_client && g_client.connected()) {
            // One session owns the adapter, same arbitration as BLE: two
            // clients interleaving commands would corrupt both, and silently,
            // because each reply is a valid reply to *some* request.
            incoming.stop();
        } else {
            g_client = incoming;
            g_client.setNoDelay(true);
            g_buffer.reset();
        }
    }

    while (g_client && g_client.available() > 0) {
        const char c = static_cast<char>(g_client.read());
        if (!g_buffer.offer(c)) continue;

        std::string reply;
        if (g_buffer.overflowed()) {
            // Command was discarded, so it must not run. Answer as the adapter
            // answers any unparseable input.
            reply = "?\r>";
        } else {
            g_handler(g_buffer.line(), reply);
        }
        // Same short-write contract as USB CDC: WiFiClient::write returns what
        // the TCP send buffer accepted, which under a slow or stalled peer is
        // less than asked. Dropping the tail loses the ">" prompt and the
        // client waits forever. See link/write_all.h.
        writeAll([](const char* data, size_t size) {
                     return g_client.write(reinterpret_cast<const uint8_t*>(data), size);
                 },
                 reply.data(), reply.size());
    }
}
