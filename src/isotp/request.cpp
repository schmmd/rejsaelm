#include "isotp/request.h"
#include <cstring>

bool buildSingleFrameRequest(const uint8_t* payload, size_t len, uint8_t out[8]) {
    if (payload == nullptr || len == 0 || len > 7) return false;
    std::memset(out, 0x00, 8);
    out[0] = static_cast<uint8_t>(len & 0x0F);
    std::memcpy(out + 1, payload, len);
    return true;
}

void buildFlowControlFrame(uint8_t out[8]) {
    std::memset(out, 0x00, 8);
    out[0] = 0x30;
}
