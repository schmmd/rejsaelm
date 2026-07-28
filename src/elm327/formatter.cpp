#include "elm327/formatter.h"
#include <cstdio>

const char* kPrompt = ">";

namespace {

constexpr size_t kBytesPerLine = 7;

void appendHex(std::string& out, uint8_t byte) {
    char buf[3];
    std::snprintf(buf, sizeof(buf), "%02X", byte);
    out += buf;
}

} // namespace

std::string formatPayload(const uint8_t* data, size_t len, const AdapterState& state) {
    std::string out;

    if (len <= kBytesPerLine) {
        for (size_t i = 0; i < len; ++i) {
            if (i && state.spaces) out += ' ';
            appendHex(out, data[i]);
        }
        return out;
    }

    // Multi-frame: three-hex-digit total length, then index-prefixed lines.
    char header[8];
    std::snprintf(header, sizeof(header), "%03X", static_cast<unsigned>(len));
    out += header;

    size_t index = 0;
    for (size_t offset = 0; offset < len; offset += kBytesPerLine, ++index) {
        out += '\r';
        char prefix[4];
        std::snprintf(prefix, sizeof(prefix), "%X:", static_cast<unsigned>(index & 0xF));
        out += prefix;

        const size_t count = (len - offset < kBytesPerLine) ? len - offset : kBytesPerLine;
        for (size_t i = 0; i < count; ++i) {
            if (state.spaces) out += ' ';
            appendHex(out, data[offset + i]);
        }
    }
    return out;
}

std::string formatFault(const char* faultText) {
    return std::string(faultText);
}

std::string formatNrc(uint8_t nrc) {
    std::string out = "NRC ";
    appendHex(out, nrc);
    return out;
}
