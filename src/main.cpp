#include <Arduino.h>
#include "board_pins.h"
#include "can/can_bus.h"
#include "elm327/session.h"
#include "link/ble_link.h"
#include "link/serial_link.h"
#include "link/wifi_link.h"

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

#if defined(BUS_TEST_ONLY)
    // Listen-only: the controller physically cannot drive the bus.
    if (!canBusBegin(true)) {
        digitalWrite(WARN_LED_PIN, HIGH);
        Serial.println("BUS TEST: TWAI init FAILED");
    } else {
        Serial.println("BUS TEST: listen-only, 500 kbit/s, accept-all");
    }
#else
    // Both real builds transmit, so CAN init is shared; only the transport
    // differs. Hoisted rather than duplicated per branch — the bus-test build
    // is the only one that opens the controller listen-only.
    if (!canBusBegin(false)) {          // NORMAL mode — we must transmit
        digitalWrite(WARN_LED_PIN, HIGH);
        Serial.println("CAN init FAILED");
    }
#  if defined(USB_SERIAL_LINK)
    serialLinkSetLineHandler(onLine);
#  else
    bleLinkSetLineHandler(onLine);
    bleLinkBegin("RejsaElm");
#    if defined(WIFI_LINK)
    // Runs ALONGSIDE BLE, not instead of it. Both feed the same session, and
    // whichever client connects first holds it until it leaves; see
    // link/session_owner.h. WPA2 rather than an open AP: anyone who associates
    // is one connect away from the bus.
    wifiLinkSetLineHandler(onLine);
    wifiLinkBegin("RejsaElm", WIFI_LINK_PASSWORD);
#    endif
#  endif
#endif
}

#if defined(BUS_TEST_ONLY)
void loop() {
    twai_message_t msg;
    while (canBusReceive(msg, 10)) {
        digitalWrite(ACTIVITY_LED_PIN, !digitalRead(ACTIVITY_LED_PIN));
    }

    static uint32_t lastReport = 0;
    if (millis() - lastReport >= 1000) {
        lastReport = millis();
        CanCounters c = canBusCounters();
        Serial.printf("frames=%lu busErrors=%lu rxMissed=%lu rxOverrun=%lu\n",
                      (unsigned long)c.received, (unsigned long)c.busErrors,
                      (unsigned long)c.rxMissed, (unsigned long)c.rxOverrun);
    }
}
#elif defined(USB_SERIAL_LINK)
void loop() { serialLinkPoll(); }
#elif defined(WIFI_LINK)
// BLE drives itself from its own task; only the WiFi link needs polling.
void loop() { wifiLinkPoll(); }
#else
void loop() { delay(1000); }
#endif
