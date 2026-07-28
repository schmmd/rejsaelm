#pragma once
#include <string>

// USB CDC transport for the ELM327 session, used by env:rejsacan_usbscan.
//
// Deliberately NOT started alongside bleLinkBegin(). Elm327Session holds
// mutable state_ and is documented as strictly half-duplex; BLE callbacks run
// on the BLE task while serialLinkPoll() runs in loop(), so feeding one session
// from both would race on state_ and on the CAN bus. The failure mode is
// silently misattributed data, not an error, so the two links are separated at
// build time instead of being synchronised at runtime.
//
// Same handler signature as ble_link.h: one complete CR-stripped line in, the
// reply text (prompt included) out.
void serialLinkSetLineHandler(void (*handler)(const char* line, std::string& reply));

// Drains everything readable on the port. Call from loop().
void serialLinkPoll();
