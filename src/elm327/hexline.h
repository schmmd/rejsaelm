#pragma once
#include <cstddef>
#include <cstdint>

// Pure parsing/classification helpers used by the diagnostic session. They live
// here rather than in session.cpp so the host test build can reach them:
// session.cpp pulls in Arduino.h and the TWAI driver and cannot compile on the
// host, and these are exactly the pieces that must not be wrong.

// Parses an ELM327 command line of hex digits ("220101", "22 01 01") into
// bytes. Embedded whitespace is ignored, as a real ELM327 ignores it.
//
// Returns false — writing nothing useful to `out` — for: an odd number of hex
// digits, any non-hex non-space character, empty input, and more bytes than
// `maxLen`. The bounds check happens BEFORE the write, so a line longer than
// the buffer cannot overflow it.
bool hexLineToBytes(const char* line, uint8_t* out, size_t maxLen, size_t& outLen);

// How a reassembled UDS payload relates to the request that was sent.
enum class ResponseMatch {
    Positive,        // service echo present: this is our answer
    NegativePending, // 7F <service> 78 — "response pending", keep waiting
    NegativeFinal,   // 7F <service> <nrc> — the ECU rejected the request
    Mismatch,        // not an answer to this request; ignore it
};

// Correlates a reassembled response with the request that produced it.
//
// UDS has exactly one correlation mechanism: a positive response echoes the
// request's service ID with bit 6 set (0x22 -> 0x62, 0x01 -> 0x41), and a
// negative response is 7F <requested service> <NRC>. Nothing else ties a reply
// to a request — no sequence numbers, no tags.
//
// The service byte alone is not selective enough: every polled request in
// this app uses service 0x22 and differs only in the two-byte DID that
// follows, so a late reply to an earlier DID would otherwise pass as the
// current one and be decoded as its data. For services 0x22 (DID, two bytes)
// and 0x01/0x09 (PID, one byte), the echoed identifier is also verified.
// Other services keep the service-only check, since this project has not
// verified their echo shape and guessing could reject a legitimate reply.
// Without this check a late reply to an earlier PID is decoded as the current
// PID's data and displayed as a valid reading, which is the one failure this
// project cannot tolerate.
//
// 0x7F is tested first because a negative response legitimately does NOT carry
// the service echo.
ResponseMatch classifyResponse(const uint8_t* request, size_t requestLen,
                               const uint8_t* response, size_t responseLen);
