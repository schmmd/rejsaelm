#pragma once
#include <cstdint>
#include <string>

// ATST bounds, in milliseconds.
//
// ATST00 asks for a 0 ms response timeout, which would make every request
// return NO DATA instantly — the adapter would look like a dead car. A real
// ELM327 substitutes its default instead of honouring 0, so the value is
// clamped into [kMinTimeoutMs, kMaxTimeoutMs].
constexpr uint16_t kMinTimeoutMs = 20;
constexpr uint16_t kMaxTimeoutMs = 65535;

// The adapter settings a client can change.
//
// NOT HONOURED — accepted for client compatibility only:
//   `headers`   (ATH0/ATH1) — the firmware ALWAYS emits the headers-off form:
//                the payload alone, with no CAN ID and no ISO-TP PCI bytes,
//                using a bare 3-hex-digit length line plus `0:`/`1:` index
//                prefixes for multi-frame replies. ATH1 is answered OK and then
//                ignored. Implementing it would mean a second output format
//                that nothing in this project reads.
//   `linefeeds` (ATL0/ATL1) — replies are always CR-terminated with no LF.
//                ATL1 is answered OK and then ignored.
// Both flags are stored so the state is inspectable and so a future
// implementation has somewhere to read from, but no code consults them today.
// A client that sets either one and depends on the result will be
// disappointed; the Android app requests exactly the hardcoded behaviour
// (ATH0/ATL0), so for it the two are identical.
//
// HONOURED: `echo` (Elm327Session::handleLine prefixes the reply with the
// command line), `spaces` (formatPayload), `header` (ATSH, the request CAN ID),
// `timeoutMs` (ATST, the response deadline).
// `autoFormat` (ATCAF) is structural: this firmware only ever does ISO-TP
// assembly, which is what ATCAF1 means, and ATCAF0 is stored but not honoured.
// The most recent frame accepted by the session, kept solely so ATBD has
// something to report. One frame, not the whole sequence: ATBD dumps a buffer,
// and retaining a full multi-frame response would be the frame-retention work
// that real ATH1 support needs and this phase deliberately does not do.
struct ReceivedFrame {
    bool valid = false;
    uint8_t dlc = 0;
    uint8_t data[8] = {};
};

struct AdapterState {
    bool echo = true;        // ATE1
    bool spaces = true;      // ATS1
    bool headers = false;    // ATH0 — stored, NOT honoured (see above)
    bool linefeeds = false;  // ATL0 — stored, NOT honoured (see above)
    bool autoFormat = true;  // ATCAF1 — stored, NOT honoured (see above)
    uint16_t header = 0x7DF; // ATSH — functional broadcast address
    uint16_t timeoutMs = 200;

    // @3 — a client-set device identifier, exactly 12 characters, reported
    // back by @2. Stored raw: this is a payload, not a command, so the
    // uppercasing and space-stripping canonical() does must not touch it.
    // A char array rather than std::string keeps AdapterState trivially
    // assignable, which is what makes `state = AdapterState{}` a valid reset.
    char identifier[13] = {};

    // ATSP — the selected protocol number. Only 6 (ISO 15765-4, CAN 11-bit,
    // 500 kbit/s) is selectable until phase 3 adds 7/8/9.
    uint8_t protocol = 6;
    // True when the client asked the adapter to choose (ATSP0 / ATSPAn)
    // rather than pinning one. ATDP/ATDPN mark this, so a client can tell a
    // negotiated protocol from one it set itself.
    bool autoSelected = false;

    bool responses = true;      // ATR1 — wait for a reply after transmitting
    bool variableDlc = false;   // ATV0 — send full 8-byte frames
    // ATAL/ATNL — STORED, NOT HONOURED. Allowing messages longer than 7 bytes
    // means multi-frame transmit, and runRequest() builds single frames only.
    // There is nothing to enable, so this flag changes no behaviour today.
    bool allowLong = false;
    uint8_t testerAddress = 0xF9;  // ATTA — the conventional OBD tester address
    uint8_t priorityBits = 0x18;   // ATCP — 29-bit ID priority, phase 3

    // ATCEA — CAN extended addressing. The address byte occupies the first
    // data byte of every frame, so the usable payload drops to 6.
    bool extendedAddressing = false;
    uint8_t extendedAddress = 0;

    ReceivedFrame lastFrame;
};

// What the caller must do after the command has been applied.
enum class AtResult {
    Ok,        // answer "OK"
    Unknown,   // answer "?"
    Reset,     // answer with the version banner
    Identify,  // answer with the version banner
    Voltage,   // answer with the measured supply voltage
    DeviceDescription,  // @1 — answer with the device description
    DeviceIdentifier,   // @2 — answer with state.identifier
    DescribeProtocol,        // ATDP  — answer with describeProtocol(state)
    DescribeProtocolNumber,  // ATDPN — answer with describeProtocolNumber(state)
    BufferDump,  // ATBD — answer with formatBufferDump(state)
    NoData,  // answer "NO DATA"
};

bool isAtCommand(const char* line);
AtResult applyAtCommand(const char* line, AdapterState& state);

// ATDP / ATDPN renderings. Pure functions of the state so they are host-tested
// rather than living as string literals inside session.cpp.
std::string describeProtocol(const AdapterState& state);
std::string describeProtocolNumber(const AdapterState& state);

// ATBD rendering: DLC first, then that many data bytes, all space-separated.
std::string formatBufferDump(const AdapterState& state);
