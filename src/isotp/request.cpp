#include "isotp/request.h"
#include <cstring>

bool buildSingleFrameRequest(const uint8_t* payload, size_t len, uint8_t out[8],
                             bool extendedAddressing, uint8_t extendedAddress) {
    const size_t maxLen = extendedAddressing ? 6 : 7;
    if (payload == nullptr || len == 0 || len > maxLen) return false;
    std::memset(out, 0x00, 8);
    uint8_t* pci = out;
    if (extendedAddressing) {
        out[0] = extendedAddress;
        pci = out + 1;
    }
    pci[0] = static_cast<uint8_t>(len & 0x0F);
    std::memcpy(pci + 1, payload, len);
    return true;
}

void buildFlowControlFrame(uint8_t out[8]) {
    std::memset(out, 0x00, 8);
    out[0] = 0x30;
}
