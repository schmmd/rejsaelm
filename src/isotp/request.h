#pragma once
#include <cstddef>
#include <cstdint>

// Builds a single-frame ISO-TP request. Returns false if the payload is empty
// or longer than 7 bytes — multi-frame requests are not needed here, and
// silently truncating one would send a different request than was asked for.
bool buildSingleFrameRequest(const uint8_t* payload, size_t len, uint8_t out[8]);

// Flow control: ContinueToSend, block size 0 (send everything), STmin 0 (no
// inter-frame delay). The ECU is the bottleneck, not us.
void buildFlowControlFrame(uint8_t out[8]);
