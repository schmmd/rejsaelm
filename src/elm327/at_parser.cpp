#include "elm327/at_parser.h"
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <string>

namespace {

// Uppercase with all whitespace removed. ELM327 ignores both.
std::string canonical(const char* line) {
    std::string out;
    for (const char* p = line; *p; ++p) {
        if (std::isspace(static_cast<unsigned char>(*p))) continue;
        out += static_cast<char>(std::toupper(static_cast<unsigned char>(*p)));
    }
    return out;
}

bool startsWith(const std::string& s, const char* prefix) {
    return s.rfind(prefix, 0) == 0;
}

// Parses the hex tail of a command like ATSH7E4 or ATST32.
bool hexTail(const std::string& s, size_t from, uint32_t& value) {
    if (from >= s.size()) return false;
    char* end = nullptr;
    value = std::strtoul(s.c_str() + from, &end, 16);
    return end && *end == '\0';
}

} // namespace

bool isAtCommand(const char* line) {
    std::string s = canonical(line);
    // '@' commands (@1/@2/@3) carry no AT prefix — see applyAtCommand.
    return startsWith(s, "AT") || startsWith(s, "@");
}

AtResult applyAtCommand(const char* line, AdapterState& state) {
    std::string s = canonical(line);
    if (!startsWith(s, "AT") && !startsWith(s, "@")) return AtResult::Unknown;

    if (s == "ATZ" || s == "ATWS") {
        state = AdapterState{};
        return AtResult::Reset;
    }
    if (s == "ATD") { state = AdapterState{}; return AtResult::Ok; }
    if (s == "ATI") return AtResult::Identify;
    if (s == "ATRV") return AtResult::Voltage;

    if (s == "ATE0") { state.echo = false; return AtResult::Ok; }
    if (s == "ATE1") { state.echo = true;  return AtResult::Ok; }
    if (s == "ATS0") { state.spaces = false; return AtResult::Ok; }
    if (s == "ATS1") { state.spaces = true;  return AtResult::Ok; }
    if (s == "ATH0") { state.headers = false; return AtResult::Ok; }
    if (s == "ATH1") { state.headers = true;  return AtResult::Ok; }
    if (s == "ATL0") { state.linefeeds = false; return AtResult::Ok; }
    if (s == "ATL1") { state.linefeeds = true;  return AtResult::Ok; }
    if (s == "ATCAF0") { state.autoFormat = false; return AtResult::Ok; }
    if (s == "ATCAF1") { state.autoFormat = true;  return AtResult::Ok; }

    if (startsWith(s, "ATSH")) {
        uint32_t v = 0;
        if (!hexTail(s, 4, v) || v > 0x7FF) return AtResult::Unknown;
        state.header = static_cast<uint16_t>(v);
        return AtResult::Ok;
    }

    if (startsWith(s, "ATST")) {
        uint32_t v = 0;
        if (!hexTail(s, 4, v)) return AtResult::Unknown;
        // ELM327 expresses ATST in 4 ms units.
        uint32_t ms = v * 4;
        // ATST00 asks for 0 ms, which would make every single request return
        // NO DATA before the car could possibly answer. A real ELM327 treats 00
        // as "use the default" rather than "never wait"; clamp to a floor so a
        // client cannot accidentally disable the adapter with one command.
        if (ms < kMinTimeoutMs) ms = kMinTimeoutMs;
        // Clamp the top end too, so a large value cannot wrap through the
        // uint16 cast back down past the floor (0x4000 * 4 = 65536 -> 0).
        if (ms > kMaxTimeoutMs) ms = kMaxTimeoutMs;
        state.timeoutMs = static_cast<uint16_t>(ms);
        return AtResult::Ok;
    }

    if (startsWith(s, "ATSP")) {
        // We speak exactly one protocol: ISO 15765-4 CAN, 11-bit, 500 kbit/s
        // (protocol 6). Accept 6 and auto (0); reject anything else rather
        // than pretending to support a bus we cannot drive.
        if (s == "ATSP6" || s == "ATSP0" || s == "ATSPA6") return AtResult::Ok;
        return AtResult::Unknown;
    }

    // Commands whose behaviour we always get right internally. Accepting them
    // is friendlier than '?', which makes clients think the adapter is broken.
    if (startsWith(s, "ATAT") || startsWith(s, "ATFC") ||
        startsWith(s, "ATCF") || startsWith(s, "ATCM") ||
        startsWith(s, "ATCRA") || s == "ATM0" || s == "ATM1") {
        return AtResult::Ok;
    }

    return AtResult::Unknown;
}
