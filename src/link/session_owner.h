#pragma once
#include <atomic>

// Which transport currently owns the single Elm327Session.
//
// PURE LOGIC, header-only, for the same reason as write_all.h: link/*.cpp is
// excluded from env:native because it pulls in the Arduino radio stacks, so the
// arbitration that decides who may touch the session has to live in a header to
// be testable on the host. Getting this wrong is silent — see below — so it is
// exactly the part that must not be inferred from bench behaviour.
//
// BLE and WiFi can both be up at once, but only one client may hold the
// session, because Elm327Session is half-duplex and stateful: it runs a whole
// request/response exchange inside handleLine() and keeps the ATSH header and
// echo/space settings between commands. A second client does not corrupt the
// first visibly; it reconfigures the adapter underneath it, and the first
// client gets a well-formed answer from the wrong ECU.
//
// Claim/release is atomic because the two links run on different tasks: BLE
// callbacks fire on the NimBLE host task, wifiLinkPoll() runs in loop(). A
// plain bool read-then-set could let both sides observe "free" in the same
// instant and both proceed.
enum class LinkId { None, Ble, Wifi };

inline std::atomic<LinkId>& sessionOwner() {
    static std::atomic<LinkId> owner{LinkId::None};
    return owner;
}

// Takes the session for `link` if nobody holds it. Returns false if someone
// does — including `link` itself, so a second client on the SAME transport is
// rejected by the same call that rejects the other transport.
inline bool sessionOwnerClaim(LinkId link) {
    LinkId expected = LinkId::None;
    return sessionOwner().compare_exchange_strong(expected, link);
}

// Releases only if `link` is in fact the holder.
//
// The conditional is load-bearing, not defensive. Rejecting a client works by
// disconnecting it, which fires the same disconnect path as a real departure —
// on BLE for the rejected handle, on WiFi for the closed socket. An
// unconditional release would let the rejected client's teardown hand the
// session away from the live owner, and the bench test would still show the
// newcomer being dropped and look correct.
inline void sessionOwnerRelease(LinkId link) {
    LinkId expected = link;
    sessionOwner().compare_exchange_strong(expected, LinkId::None);
}

inline bool sessionOwnerIs(LinkId link) { return sessionOwner().load() == link; }
