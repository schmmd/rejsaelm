#pragma once
#include <string>

// TCP transport for the ELM327 session, used by env:rejsacan_wifi.
//
// The board runs its own access point and listens on port 35000 — the
// convention every ELM327-over-WiFi clone uses, so SavvyCAN, OBD Fusion and a
// plain `nc 192.168.4.1 35000` all work without configuration. Station mode
// (joining an existing network) is deliberately not offered: the car is not
// where the house WiFi is, and credentials would need somewhere to live.
//
// Started INSTEAD of BLE, not alongside it, for the reason spelled out in
// serial_link.h: Elm327Session is half-duplex and holds mutable state, and BLE
// callbacks run on the NimBLE host task while wifiLinkPoll() runs in loop().
// Two links feeding one session would race on that state and on the CAN bus.
//
// Same handler signature as ble_link.h: one complete CR-stripped line in, the
// reply text (prompt included) out.
void wifiLinkSetLineHandler(void (*handler)(const char* line, std::string& reply));

// Brings up the access point and starts listening. `password` must be at least
// 8 characters — WPA2's minimum, and an open AP would put the CAN bus of a
// parked car within reach of anyone in radio range.
void wifiLinkBegin(const char* ssid, const char* password);

// Accepts connections and drains whatever is readable. Call from loop().
void wifiLinkPoll();
