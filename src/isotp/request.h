#pragma once
#include <cstddef>
#include <cstdint>

// Builds a single-frame ISO-TP request. Returns false if the payload is empty
// or does not fit — multi-frame requests are not needed here, and silently
// truncating one would send a different request than was asked for.
//
// With extended addressing (ATCEA) the address byte occupies out[0], so the
// usable payload drops from 7 bytes to 6. The default arguments keep every
// existing caller building exactly the frame it built before.
bool buildSingleFrameRequest(const uint8_t* payload, size_t len, uint8_t out[8],
                             bool extendedAddressing = false,
                             uint8_t extendedAddress = 0);

// Flow control: ContinueToSend, block size 0 (send everything), STmin 0 (no
// inter-frame delay). The ECU is the bottleneck, not us.
void buildFlowControlFrame(uint8_t out[8]);
