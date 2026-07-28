#pragma once
#include <cstdint>

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
struct AdapterState {
    bool echo = true;        // ATE1
    bool spaces = true;      // ATS1
    bool headers = false;    // ATH0 — stored, NOT honoured (see above)
    bool linefeeds = false;  // ATL0 — stored, NOT honoured (see above)
    bool autoFormat = true;  // ATCAF1 — stored, NOT honoured (see above)
    uint16_t header = 0x7DF; // ATSH — functional broadcast address
    uint16_t timeoutMs = 200;
};

// What the caller must do after the command has been applied.
enum class AtResult {
    Ok,        // answer "OK"
    Unknown,   // answer "?"
    Reset,     // answer with the version banner
    Identify,  // answer with the version banner
    Voltage,   // answer with the measured supply voltage
};

bool isAtCommand(const char* line);
AtResult applyAtCommand(const char* line, AdapterState& state);
