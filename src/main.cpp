#include <Arduino.h>
#include "board_pins.h"
#include "can/can_bus.h"
#include "elm327/session.h"
#include "link/ble_link.h"
#include "link/serial_link.h"
#include "link/wifi_link.h"

#if defined(WIFI)
#  ifndef WIFI_PASSWORD
// A default so -DWIFI alone builds, and a warning because a published default
// password on a device wired to a CAN bus is not a default anyone should ship.
#    define WIFI_PASSWORD "rejsacan"
#    warning "Building WiFi with the default AP password. Override with -DWIFI_PASSWORD"
#  endif
// WPA2 rejects anything shorter and softAP() would simply fail to come up at
// runtime, which looks like a broken radio rather than a bad flag.
static_assert(sizeof(WIFI_PASSWORD) >= 9, "WIFI_PASSWORD must be at least 8 characters");
#endif

static Elm327Session g_session;

static void onLine(const char* line, std::string& reply) {
    digitalWrite(ACTIVITY_LED_PIN, HIGH);
    reply = g_session.handleLine(line);
    digitalWrite(ACTIVITY_LED_PIN, LOW);
}

void setup() {
    Serial.begin(115200);
    pinMode(FORCE_ON_PIN, OUTPUT);
    digitalWrite(FORCE_ON_PIN, HIGH);
    pinMode(WARN_LED_PIN, OUTPUT);
    pinMode(ACTIVITY_LED_PIN, OUTPUT);

    // Every build transmits; only the transport differs.
    if (!canBusBegin(false)) {          // NORMAL mode — we must transmit
        digitalWrite(WARN_LED_PIN, HIGH);
        Serial.println("CAN init FAILED");
    }
#if defined(USB_SERIAL_LINK)
    serialLinkSetLineHandler(onLine);
#else
    bleLinkSetLineHandler(onLine);
    bleLinkBegin("RejsaElm");
#  if defined(WIFI)
    // Runs ALONGSIDE BLE, not instead of it. Both feed the same session, and
    // whichever client connects first holds it until it leaves; see
    // link/session_owner.h. WPA2 rather than an open AP: anyone who associates
    // is one connect away from the bus.
    wifiLinkSetLineHandler(onLine);
    wifiLinkBegin("RejsaElm", WIFI_PASSWORD);
#  endif
#endif
}

#if defined(USB_SERIAL_LINK)
void loop() { serialLinkPoll(); }
#elif defined(WIFI)
// BLE drives itself from its own task; only the WiFi link needs polling.
void loop() { wifiLinkPoll(); }
#else
void loop() { delay(1000); }
#endif
