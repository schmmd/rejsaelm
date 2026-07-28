#pragma once
#include <string>

// Starts BLE advertising with both the ELM327 clone profile (FFE0/FFE1) and
// the Nordic UART Service. Both feed the same line handler.
void bleLinkBegin(const char* deviceName);

// The handler receives one complete command line (CR stripped) and fills
// `reply` with the text to send back, prompt included.
void bleLinkSetLineHandler(void (*handler)(const char* line, std::string& reply));
