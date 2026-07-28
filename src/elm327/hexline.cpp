#include "elm327/hexline.h"
#include <cctype>

bool hexLineToBytes(const char* line, uint8_t* out, size_t maxLen, size_t& outLen) {
    outLen = 0;
    if (!line) return false;
    int high = -1;
    for (const char* p = line; *p; ++p) {
        if (std::isspace(static_cast<unsigned char>(*p))) continue;
        if (!std::isxdigit(static_cast<unsigned char>(*p))) return false;
        const int value = std::isdigit(static_cast<unsigned char>(*p))
            ? *p - '0'
            : std::toupper(static_cast<unsigned char>(*p)) - 'A' + 10;
        if (high < 0) {
            high = value;
        } else {
            if (outLen >= maxLen) return false;   // bounds check before the write
            out[outLen++] = static_cast<uint8_t>((high << 4) | value);
            high = -1;
        }
    }
    return high < 0 && outLen > 0;  // reject an odd number of hex digits
}

ResponseMatch classifyResponse(const uint8_t* request, size_t requestLen,
                               const uint8_t* response, size_t responseLen) {
    if (!request || !response || requestLen == 0 || responseLen == 0) {
        return ResponseMatch::Mismatch;
    }

    if (response[0] == 0x7F) {
        // 7F <service> <NRC>. Anything shorter is malformed; treat it as not
        // ours rather than guessing which request it rejects.
        if (responseLen < 3) return ResponseMatch::Mismatch;
        if (response[1] != request[0]) return ResponseMatch::Mismatch;
        // 0x78 = requestCorrectlyReceived-ResponsePending. The real answer
        // still follows, so the caller keeps waiting.
        return response[2] == 0x78 ? ResponseMatch::NegativePending
                                   : ResponseMatch::NegativeFinal;
    }

    if (response[0] != static_cast<uint8_t>(request[0] + 0x40)) {
        return ResponseMatch::Mismatch;
    }

    // The service byte alone does not identify which request this answers:
    // every polled request in this app uses service 0x22, differing only in
    // the DID that follows, so a late reply to an earlier PID would otherwise
    // pass as the current one. Where UDS defines an echoed identifier, verify
    // it too. For services without a known echo shape (e.g. write/control
    // services 0x2E, 0x2F, 0x31), keep the service-only check rather than
    // guess a shape that might reject a legitimate reply.
    switch (request[0]) {
        case 0x22:  // ReadDataByIdentifier: two-byte DID echoed back.
            if (requestLen < 3 || responseLen < 3) return ResponseMatch::Mismatch;
            if (response[1] != request[1] || response[2] != request[2]) {
                return ResponseMatch::Mismatch;
            }
            break;
        case 0x01:  // Show current data
        case 0x09:  // Request vehicle information
            if (requestLen < 2 || responseLen < 2) return ResponseMatch::Mismatch;
            if (response[1] != request[1]) return ResponseMatch::Mismatch;
            break;
        default:
            break;
    }

    return ResponseMatch::Positive;
}
