#include "elm327/at_parser.h"
#include <cctype>
#include <cstdio>
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

// Parses a one-byte hex tail, e.g. ATTAF9. Rejects values that do not fit in
// a byte rather than truncating them into a different address.
bool byteTail(const std::string& s, size_t from, uint8_t& value) {
    uint32_t v = 0;
    if (!hexTail(s, from, v) || v > 0xFF) return false;
    value = static_cast<uint8_t>(v);
    return true;
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

    if (s == "@1") return AtResult::DeviceDescription;
    if (s == "@2") {
        // Nothing has been set, so there is nothing truthful to report.
        return state.identifier[0] ? AtResult::DeviceIdentifier
                                   : AtResult::Unknown;
    }
    if (startsWith(s, "@3")) {
        // Parse the RAW line: the identifier is a payload and canonical()
        // has already destroyed its case and spacing.
        const char* p = std::strchr(line, '3');
        if (!p) return AtResult::Unknown;
        ++p;
        while (*p == ' ') ++p;
        const size_t len = std::strlen(p);
        // ELM327 specifies exactly 12 characters. Accepting a short value
        // would leave the rest of the field as stale bytes from a previous set.
        if (len != 12) return AtResult::Unknown;
        std::memcpy(state.identifier, p, 12);
        state.identifier[12] = '\0';
        return AtResult::Ok;
    }

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
        // (protocol 6). Phase 3 adds 7/8/9. Anything else is rejected rather
        // than accepted and silently not driven.
        // This block IS order-sensitive: it returns from inside itself for
        // every case, including the rejection, so it must stay ahead of
        // anything else that might otherwise want to claim an "ATSP..." line.
        // It never falls through to the catch-all Unknown at the bottom.
        if (s == "ATSP6")   { state.protocol = 6; state.autoSelected = false; return AtResult::Ok; }
        if (s == "ATSP0")   { state.protocol = 6; state.autoSelected = true;  return AtResult::Ok; }
        if (s == "ATSPA6" || s == "ATSPA0") {
            state.protocol = 6; state.autoSelected = true; return AtResult::Ok;
        }
        return AtResult::Unknown;
    }

    if (s == "ATDP")  return AtResult::DescribeProtocol;
    if (s == "ATDPN") return AtResult::DescribeProtocolNumber;

    if (s == "ATR0") { state.responses = false; return AtResult::Ok; }
    if (s == "ATR1") { state.responses = true;  return AtResult::Ok; }
    if (s == "ATV0") { state.variableDlc = false; return AtResult::Ok; }
    if (s == "ATV1") { state.variableDlc = true;  return AtResult::Ok; }
    if (s == "ATAL") { state.allowLong = true;  return AtResult::Ok; }
    if (s == "ATNL") { state.allowLong = false; return AtResult::Ok; }

    if (startsWith(s, "ATTA")) {
        uint8_t v = 0;
        if (!byteTail(s, 4, v)) return AtResult::Unknown;
        state.testerAddress = v;
        return AtResult::Ok;
    }
    // ATCEA is an exact/prefix match on its own five-letter name; it shares no
    // prefix with ATCF/ATCM/ATCRA below, so there is no ordering dependency
    // between this block and that one — either could move without effect.
    if (startsWith(s, "ATCEA")) {
        // A bare ATCEA turns extended addressing off.
        if (s == "ATCEA") { state.extendedAddressing = false; return AtResult::Ok; }
        uint8_t v = 0;
        if (!byteTail(s, 5, v)) return AtResult::Unknown;
        state.extendedAddressing = true;
        state.extendedAddress = v;
        return AtResult::Ok;
    }
    if (startsWith(s, "ATCP")) {
        uint8_t v = 0;
        if (!byteTail(s, 4, v)) return AtResult::Unknown;
        state.priorityBits = v;
        return AtResult::Ok;
    }

    if (s == "ATBD") {
        return state.lastFrame.valid ? AtResult::BufferDump : AtResult::Unknown;
    }

    // Commands whose behaviour we always get right internally. Accepting them
    // is friendlier than '?', which makes clients think the adapter is broken.
    if (startsWith(s, "ATAT") || startsWith(s, "ATFC") ||
        startsWith(s, "ATCF") || startsWith(s, "ATCM") ||
        startsWith(s, "ATCRA") || s == "ATM0" || s == "ATM1") {
        return AtResult::Ok;
    }

    if (s == "ATPC") return AtResult::Ok;

    // Monitor modes stream frames until interrupted, which the session cannot
    // do yet — handleLine() is strictly request/reply/prompt. Phase 4 builds
    // the streaming path and these become real. Answer immediately meanwhile.
    // No ordering dependency on the ATM0/ATM1 entries above: those are exact
    // matches on "ATM0"/"ATM1", which cannot match the ATMR/ATMT prefixes or
    // the ATMA literal here, so this block could move without effect. Matched
    // by exact/4-char prefix rather than a bare "ATM" prefix so J1939's ATMP
    // (monitor for PGN) is not swallowed here — it is a deferred command that
    // must still fall through to Unknown below.
    if (s == "ATMA" || startsWith(s, "ATMR") || startsWith(s, "ATMT")) {
        return AtResult::NoData;
    }

    return AtResult::Unknown;
}

std::string describeProtocol(const AdapterState& state) {
    // One protocol until phase 3 adds 7/8/9; a switch here then.
    std::string out = state.autoSelected ? "AUTO, " : "";
    out += "ISO 15765-4 (CAN 11/500)";
    return out;
}

std::string describeProtocolNumber(const AdapterState& state) {
    std::string out = state.autoSelected ? "A" : "";
    out += static_cast<char>('0' + state.protocol);
    return out;
}

std::string formatBufferDump(const AdapterState& state) {
    char buf[4];
    std::snprintf(buf, sizeof(buf), "%02X", state.lastFrame.dlc);
    std::string out = buf;
    // Only up to the DLC: the tail of the array is whatever was there before,
    // not something the ECU sent.
    for (uint8_t i = 0; i < state.lastFrame.dlc && i < 8; ++i) {
        std::snprintf(buf, sizeof(buf), "%02X", state.lastFrame.data[i]);
        out += ' ';
        out += buf;
    }
    return out;
}
