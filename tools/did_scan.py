#!/usr/bin/env python3
"""Read-only UDS DID scanner for a Hyundai IONIQ 5.

Requires the board to be running env:rejsacan_usbscan:

    cd esp32-s3 && pio run -e rejsacan_usbscan -t upload

SAFETY — these are not options:
  * The ONLY UDS service sent is 22 (ReadDataByIdentifier). Enforced in
    Adapter.send_did(). Services 2E/2F/31/11/27/34/36 are never constructed.
  * The default diagnostic session is never left: no 10 03, no 3E.
  * Only physical ECU addresses are probed, never the functional address 7DF,
    because session.cpp accepts a frame from ANY ecu on 7DF and two responders
    would interleave into a corrupt reassembly.

Run the car in Ready mode, parked, not charging.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone

import serial  # pyserial

from didscan_core import (
    ADAPTER_ERROR,
    DEFAULT_RANGES,
    NEGATIVE,
    NO_RESPONSE,
    POSITIVE,
    RangeError,
    Reply,
    already_scanned,
    classify,
    expand_ranges,
    recorded_ecus,
)

BANNER = "ELM327 v1.5"
PROMPT = b">"

# ATST's floor in at_parser.h. Below this the firmware silently substitutes its
# own value, so the tool rejects it rather than reporting a timeout it is not
# actually using.
MIN_TIMEOUT_MS = 20

# ATST's ceiling in at_parser.h (kMaxTimeoutMs). Above this the firmware clamps,
# for the same reason it clamps below the floor, so the tool refuses it for the
# same reason too: the timeout it reports has to be the one the board is using.
MAX_TIMEOUT_MS = 65535

# Never probed: on 7DF the firmware accepts any responder.
FUNCTIONAL_ADDRESS = 0x7DF

# The address block discovery sweeps, inclusive at both ends.
#
# 0x7E7 is the top of the standard OBD-II physical request block, and that block
# is where the modules this project exists to read actually live: 0x7E4 is the
# BMS (state of charge, pack voltage, cell temperatures) and 0x7E5 the ICCU.
#
# THE UPPER BOUND AND THE FORBIDDEN ADDRESS ARE DIFFERENT IDEAS AND MUST NOT
# SHARE AN EXPRESSION. The pre-fix code wrote the bound as
# range(0x700, FUNCTIONAL_ADDRESS), reading it as "stop before the address we
# must never probe" -- but 0x7DF sits numerically BELOW 0x7E0, so that one
# expression silently excluded the whole 0x7E0-0x7E7 block, and a full run
# reported nothing at all for the BMS while printing a progress display and a
# CSV that read as a completed whole-car pass. The exclusion of 0x7DF is a
# filter over the candidates (see discovery_candidates), never a range bound.
DISCOVERY_FIRST_ADDRESS = 0x700
DISCOVERY_LAST_ADDRESS = 0x7E7

# Slack over the board's own response deadline before pyserial gives up waiting.
# Covers the USB round trip and the board's own line handling; the board must
# always be the thing that decides a request has timed out, because a pyserial
# timeout instead surfaces as an AdapterError that ends the whole run.
SERIAL_READ_MARGIN_S = 2.0


class AdapterError(RuntimeError):
    pass


def serial_timeout_s(timeout_ms: int, read_timeout_ms: int) -> float:
    """How long pyserial should wait for a byte, given the board's deadlines.

    Both ATST values matter: the sweep runs at --timeout, Phase 4b and
    discovery run at --read-timeout, and whichever is larger is how long the
    board may legitimately stay quiet before answering. A fixed 5 s (the old
    default) meant --read-timeout 6000 was accepted, the board waited 6000 ms,
    pyserial gave up at 5000, and the resulting AdapterError escaped sweep() to
    be printed as "init failed" -- with the recorder's end/close never running.
    """
    return max(timeout_ms, read_timeout_ms) / 1000.0 + SERIAL_READ_MARGIN_S


def open_serial(port: str, baud: int = 115200, read_timeout_s: float = 5.0):
    """Open the real port. The only place pyserial is instantiated."""
    return serial.Serial(port, baud, timeout=read_timeout_s)


class Adapter:
    """The ELM327 conversation. The only code that does I/O.

    Takes an ALREADY-OPEN stream rather than a port name, so the tests can drive
    it with a FakeBoard instead of hardware. Without that seam the entire
    request/response path — read-to-prompt framing, the service-22 assertion,
    header caching — would have no automated coverage at all.

    The stream must provide five methods: reset_input_buffer, write, flush,
    read(n) returning b"" on timeout, and close.
    """

    def __init__(self, stream):
        self._serial = stream
        self._header: str | None = None

    def close(self) -> None:
        self._serial.close()

    def forget_header(self) -> None:
        """Invalidate the cached ATSH header.

        ATZ resets the board's own header back to the functional default
        0x7DF (at_parser.h: `header = 0x7DF`), independently of this cache.
        Called by initialise() so a re-init (e.g. a recovery path) can't
        leave a stale cache that makes a later send_did() skip ATSH and
        transmit on whatever address the board reset to — which for 7DF
        means the firmware accepts a reply from ANY ecu, and two responders
        would interleave into a corrupt reassembly.
        """
        self._header = None

    def _exchange(self, line: str) -> tuple[str, float]:
        """Send one line, read up to the prompt, return (reply, elapsed_ms)."""
        self._serial.reset_input_buffer()
        # Start the clock before the write: the round trip that calibrate()
        # measures includes transmission, and a board that stalls before
        # answering must show up as elapsed time, not as zero.
        started = time.perf_counter()
        self._serial.write(line.encode("ascii") + b"\r")
        self._serial.flush()

        buffer = bytearray()
        while True:
            chunk = self._serial.read(1)
            if not chunk:
                raise AdapterError(
                    f"timed out waiting for the prompt after {line!r}; "
                    f"got {bytes(buffer)!r}"
                )
            buffer += chunk
            if chunk == PROMPT:
                break
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        reply = buffer.decode("ascii", errors="replace")

        # The adapter's echo flag is captured BEFORE the command is applied
        # (session.cpp), so ATE0 is itself echoed even though it turns echo
        # off for everything after it — and echo, when on, applies uniformly
        # to AT and non-AT lines alike, not just AT ones. Strip a leading
        # echo of this exact line here, in the one place every reply passes
        # through, so neither send_at's "OK" comparison nor classify()'s
        # line-shape parsing has to know whether echo happened to be on.
        prefix = line + "\r"
        if reply.startswith(prefix):
            reply = reply[len(prefix):]
        return reply, elapsed_ms

    def send_at(self, command: str) -> str:
        if not command.upper().startswith("AT"):
            raise AdapterError(f"{command!r} is not an AT command")
        reply, _ = self._exchange(command)
        return reply.strip().rstrip(">").strip()

    def send_did(self, ecu: str, did: int) -> tuple[Reply, float]:
        """Read one DID from one ECU.

        THE SAFETY CHOKEPOINT. Every request the tool sends is built here, and
        it can only ever be service 22.
        """
        try:
            address = int(ecu, 16)
        except ValueError as exc:
            # Every rejection this function makes has to arrive as the same
            # AdapterError the rest of the tool already catches. A bare
            # ValueError from `--ecu 7g4` would come out as a raw traceback
            # from the one function whose error contract must be uniform,
            # because it is the one place a request is ever built.
            raise AdapterError(f"{ecu!r} is not a hex ECU address") from exc
        if address == FUNCTIONAL_ADDRESS:
            raise AdapterError("refusing to probe the functional address 7DF")
        if not 0x0000 <= did <= 0xFFFF:
            raise AdapterError(f"DID {did:#x} out of range")

        if self._header != ecu:
            if self.send_at(f"ATSH{ecu}") != "OK":
                raise AdapterError(f"ATSH{ecu} was not accepted")
            self._header = ecu

        request = f"22{did:04X}"
        assert request.startswith("22"), "read-only: service 22 only"
        reply, elapsed_ms = self._exchange(request)
        return classify(reply), elapsed_ms


def initialise(adapter: Adapter, timeout_ms: int) -> None:
    """Phase 1. Reset, quieten the adapter, set the response deadline."""
    # ATZ (below) resets the board's header to 7DF, so any cached ATSH from
    # a previous session is now wrong; forget it before that reset happens
    # on the wire.
    adapter.forget_header()
    banner = adapter.send_at("ATZ")
    if BANNER not in banner:
        raise AdapterError(
            f"expected {BANNER!r} in the reset banner, got {banner!r}. "
            "Is the board running env:rejsacan_usbscan?"
        )
    for command, expected in (("ATE0", "OK"), ("ATS0", "OK"), ("ATSP6", "OK")):
        if adapter.send_at(command) != expected:
            raise AdapterError(f"{command} was not accepted")
    if adapter.send_at(f"ATST{timeout_ms // 4:02X}") != "OK":
        raise AdapterError("ATST was not accepted")


@contextlib.contextmanager
def generous_timeout(adapter: Adapter, *, timeout_ms: int,
                     read_timeout_ms: int):
    """Run a block at the wider of --read-timeout and --timeout, then put the
    sweep deadline back.

    session.cpp:76-78 sets ONE deadline for the whole receive-and-reassemble
    loop, so a multi-frame reply that needs a flow-control round trip has to
    finish inside ATST in its entirety or the firmware answers NO DATA -- which
    the host cannot tell from an ECU that said nothing. The sweep deadline is
    deliberately tiny (50 ms by default, and the docs recommend 20 for long
    runs) because it is the per-miss cost; any probe whose *expected* answer is
    a long payload must therefore be made at the generous deadline instead.

    Nothing orders --timeout against --read-timeout -- both are independently
    validated against MIN/MAX_TIMEOUT_MS, so an inverted pair like
    `--timeout 1000 --read-timeout 100` passes. Setting ATST from
    read_timeout_ms alone would then run discovery and the liveness probe
    BELOW the sweep deadline, silently reintroducing the whole-ECU version of
    the per-DID bug this context manager exists to fix. max() makes this
    context manager only ever widen the deadline, mirroring
    serial_timeout_s's max() above for the same unordered-flags reason.

    Restored in a finally, so a ScanAborted raised inside the block cannot
    leave the board on the generous value for whatever runs next. The value
    restored is always the sweep deadline (timeout_ms) itself, inverted pair
    or not -- that is what the sweep's per-miss cost and its ETA were
    calculated from.
    """
    generous_ms = max(timeout_ms, read_timeout_ms)
    adapter.send_at(f"ATST{generous_ms // 4:02X}")
    try:
        yield
    finally:
        adapter.send_at(f"ATST{timeout_ms // 4:02X}")


# Identification DIDs, tried in order during discovery. F190 is the standard VIN
# and the most widely implemented; F100 catches modules that serve an
# identification block but not F190.
DISCOVERY_DIDS = (0xF190, 0xF100)

# Certain to be unsupported, so the reply reveals how this ECU reports a miss.
CALIBRATION_DID = 0xFFFF

MISS_NRC = "NRC"          # answers promptly with a negative response code
MISS_TIMEOUT = "TIMEOUT"  # says nothing; every miss costs the full ATST


@dataclass
class EcuInfo:
    ecu: str
    vin: str | None
    miss_mode: str
    miss_ms: float
    # The discovery DID this ECU answered POSITIVE to (F190 or F100), if any.
    # sweep()'s sleep/liveness check re-probes this DID rather than an
    # arbitrary one, because it is the one DID we already know this specific
    # ECU answers. None when discovery only ever saw a NEGATIVE from this
    # ECU (nothing it answers positively is known) or when the ECU came from
    # --ecu rather than discovery.
    live_did: int | None = None
    # True once miss_mode/miss_ms are a measurement rather than a placeholder.
    # calibrate() sets it, and so does a row rebuilt from the log by --resume,
    # which is what lets a resumed run skip Phase 3 as well as Phase 2. Kept
    # explicit rather than inferred from "miss_ms > 0", because a genuinely
    # instant miss is a measurement too.
    calibrated: bool = False


def _decode_vin(payload: bytes) -> str | None:
    """Pull an ASCII VIN out of a 62 F1 90 response, if it looks like one."""
    if len(payload) < 4:
        return None
    text = payload[3:].decode("ascii", errors="ignore").strip("\x00 ")
    return text if len(text) == 17 and text.isalnum() else None


def discovery_candidates() -> list[str]:
    """Every address Phase 2 probes, as ATSH-ready hex strings.

    Two separate ideas, deliberately in two separate expressions:

      * the scan's UPPER BOUND is DISCOVERY_LAST_ADDRESS, chosen to include the
        standard OBD-II physical request block 0x7E0-0x7E7;
      * the FORBIDDEN address is 0x7DF and only 0x7DF, filtered out here.

    See DISCOVERY_LAST_ADDRESS for why collapsing the two into one range bound
    lost the BMS and the ICCU from every run.
    """
    return [f"{address:03X}"
            for address in range(DISCOVERY_FIRST_ADDRESS,
                                 DISCOVERY_LAST_ADDRESS + 1)
            if address != FUNCTIONAL_ADDRESS]


def discover_ecus(adapter: Adapter, candidates: list[str], *,
                  timeout_ms: int, read_timeout_ms: int) -> list[EcuInfo]:
    """Phase 2. Find which addresses answer at all, and grab a VIN if offered.

    Cheap — a few hundred probes — and it keeps the sweep from spending time on
    addresses with nothing behind them.

    Runs at --read-timeout, not the sweep deadline. F190 is the VIN: a ~20-byte
    reply, so multi-frame, so it needs a flow-control round trip inside the one
    deadline session.cpp sets for the whole reassembly. At the sweep value that
    can time out into NO DATA, the host reads NO_RESPONSE, and an entire ECU is
    dropped from the run with no diagnostic at all. Paying the generous deadline
    on every silent address costs a few minutes once, at the start of a run that
    lasts hours; losing the BMS costs the run.
    """
    with generous_timeout(adapter, timeout_ms=timeout_ms,
                          read_timeout_ms=read_timeout_ms):
        return _discover(adapter, candidates)


def _discover(adapter: Adapter, candidates: list[str]) -> list[EcuInfo]:
    """discover_ecus' loop, with the ATST handling hoisted out of it."""
    found: list[EcuInfo] = []
    for ecu in candidates:
        info: EcuInfo | None = None
        for did in DISCOVERY_DIDS:
            reply, _ = adapter.send_did(ecu, did)
            if reply.status == POSITIVE:
                # Nothing more to learn once something answers positively;
                # this overwrites any earlier NEGATIVE record for the same
                # ECU with the more informative one.
                vin = _decode_vin(reply.payload) if did == 0xF190 else None
                info = EcuInfo(ecu, vin, MISS_TIMEOUT, 0.0, live_did=did)
                print(f"  {ecu}: responds"
                      + (f", VIN {vin}" if vin else ""))
                break
            if reply.status == NEGATIVE:
                # A negative response still proves something is listening --
                # but it is exactly how a module says "not this DID", which
                # is the case F100 exists to catch. Record that the ECU is
                # alive (once) and keep trying the next discovery DID rather
                # than stopping here, so a module that rejects F190 still
                # gets its identification block read via F100.
                if info is None:
                    info = EcuInfo(ecu, None, MISS_NRC, 0.0)
                    print(f"  {ecu}: responds (NRC {reply.nrc:02X})")
        if info is not None:
            found.append(info)
    return found


def calibrate(adapter: Adapter, info: EcuInfo, timeout_ms: int) -> EcuInfo:
    """Phase 3. Measure how this ECU reports an unsupported DID.

    Miss cost dominates total runtime, because almost every DID is unsupported.
    An ECU that answers NRC 31 promptly is fast to sweep; one that stays silent
    costs the full ATST window on every probe. Measuring beats guessing, and
    the answer differs per module.
    """
    reply, elapsed_ms = adapter.send_did(info.ecu, CALIBRATION_DID)
    if reply.status == NEGATIVE:
        mode, cost_ms = MISS_NRC, elapsed_ms
    elif reply.status == NO_RESPONSE:
        mode, cost_ms = MISS_TIMEOUT, elapsed_ms
    elif reply.status == POSITIVE:
        # FFFF answering at all breaks the "certain to be unsupported"
        # assumption CALIBRATION_DID relies on. elapsed_ms here is a prompt
        # reply's timing, not a genuine miss cost -- recording it would
        # UNDERESTIMATE the sweep, and an underestimated ETA is worse than no
        # ETA at all, because the operator plans a session around that
        # number. Charge the pessimistic ATST window instead -- what a real
        # miss would cost -- and say so, rather than letting main()'s
        # per-ECU line render this identically to a genuine timeout.
        print(f"  {info.ecu}: unexpected POSITIVE reply to FFFF; "
              f"assuming the pessimistic ATST cost")
        mode, cost_ms = MISS_TIMEOUT, float(timeout_ms)
    else:
        # ADAPTER_ERROR: a garbled/unparseable reply to FFFF. Same reasoning
        # as the POSITIVE case -- this probe told us nothing about a genuine
        # miss's cost, so it must not be lumped in as if nothing were wrong,
        # and must not be charged its own (fast) elapsed time either.
        print(f"  {info.ecu}: unexpected malformed reply to FFFF; "
              f"assuming the pessimistic ATST cost")
        mode, cost_ms = MISS_TIMEOUT, float(timeout_ms)
    return EcuInfo(info.ecu, info.vin, mode, cost_ms, live_did=info.live_did,
                   calibrated=True)


def format_eta(seconds: float) -> str:
    if seconds < 90:
        return f"{seconds:.0f}s"
    if seconds < 5400:
        return f"{seconds / 60:.0f}m"
    return f"{seconds / 3600:.1f}h"


class Recorder:
    """Append-only JSONL writer, flushed per row.

    Flushing every row is the point: a run killed at any moment leaves a file
    that --resume can pick up from, losing at most the row in flight.
    """

    def __init__(self, path: str):
        self._file = open(path, "a", encoding="utf-8")

    def close(self) -> None:
        self._file.close()

    def _write(self, record: dict) -> None:
        self._file.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._file.flush()

    def run(self, *, firmware: str, timeout_ms: int, read_timeout_ms: int,
            ranges: list[str], full: bool) -> None:
        self._write({
            "type": "run",
            "started": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "fw": firmware,
            "timeout_ms": timeout_ms,
            "read_timeout_ms": read_timeout_ms,
            "ranges": ranges,
            "full": full,
        })

    def ecu(self, info: EcuInfo) -> None:
        self._write({
            "type": "ecu",
            "ecu": info.ecu,
            "vin": info.vin,
            "miss_mode": info.miss_mode,
            "miss_ms": round(info.miss_ms, 1),
            # Written so --resume can rebuild this ECU without rediscovering
            # it. sweep()'s liveness check re-probes live_did to tell a
            # sleeping module from a sparse one; a resumed run that had to
            # invent the ECU from scratch would lose that check silently.
            "live_did": (None if info.live_did is None
                         else f"{info.live_did:04X}"),
        })

    def did(self, ecu: str, did: int, reply: Reply, *,
            unstable: bool = False, unconfirmed: bool = False) -> None:
        record: dict = {
            "type": "did",
            "ecu": ecu,
            "did": f"{did:04X}",
            "status": reply.status,
        }
        if reply.payload is not None:
            record["len"] = len(reply.payload)
            record["payload"] = reply.payload.hex().upper()
        if reply.nrc is not None:
            record["nrc"] = f"{reply.nrc:02X}"
        if reply.status == ADAPTER_ERROR:
            record["raw"] = reply.raw
        if unstable:
            record["unstable"] = True
        if unconfirmed:
            record["unconfirmed"] = True
        self._write(record)

    def end(self, counts: dict) -> None:
        self._write({
            "type": "end",
            "completed": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            **counts,
        })


# Three consecutive adapter errors means the adapter is unhappy and everything
# after this point would be junk rather than findings.
MAX_CONSECUTIVE_ERRORS = 3

# How many times a single DID is probed before its ADAPTER_ERROR is believed.
# Observed on a real car: CAN ERROR arrives in bursts spanning several
# consecutive DIDs, and those same DIDs answer NRC 31 cleanly when a later
# --resume re-probes them. Nothing is wrong with the DIDs -- the bus is briefly
# unhealthy -- so a verdict taken from one burst is a verdict about timing.
ADAPTER_ERROR_ATTEMPTS = 3

# Spacing is what makes the extra attempts worth anything. Two probes issued
# back to back land inside the same fault and re-confirm it; the pause gives
# the transceiver time to recover first. A full second rather than a token
# delay, because the bursts seen on the car spanned several DIDs -- tens of
# milliseconds apart -- so anything shorter retries inside the same burst.
# Paid only after an error, so it costs nothing on the miss-dominated common
# path; a DID that errors on every attempt costs two seconds before the run
# gives up on it.
ADAPTER_RETRY_PAUSE_S = 1.0

# A module that answered during discovery and then goes quiet for this many
# probes in a row has almost certainly gone to sleep. Stopping beats recording
# thousands of false NO_RESPONSE rows that look like real findings.
SLEEP_SUSPICION_PROBES = 200


class ScanAborted(RuntimeError):
    pass


def sweep(adapter: Adapter, ecus: list[EcuInfo], dids: list[int],
          recorder: Recorder, *, timeout_ms: int, read_timeout_ms: int,
          skip: set[tuple[str, int]]) -> dict:
    """Phases 4 and 4b: probe every DID, then re-read each hit."""
    counts = {"probes": 0, "positive": 0, "negative": 0,
              "no_response": 0, "adapter_error": 0, "session_gated": 0,
              "skipped": len(skip), "unstable": 0, "unconfirmed": 0}
    consecutive_errors = 0
    started = time.perf_counter()

    # The ATST the board is currently set to. Normally the sweep deadline, but
    # the generous blocks below widen it, and a recovery inside one of those
    # has to put back the value that block is relying on rather than the sweep
    # value -- otherwise a stall during a multi-frame re-read would silently
    # drop the deadline to 50 ms and turn the rest of that block's replies into
    # NO DATA.
    deadline_ms = timeout_ms

    def probe(ecu: str, did: int) -> Reply:
        """Send one DID, surviving a board that stops answering.

        A stall is not a finding and not a fault of the DID: the ESP32-S3 stops
        writing to USB CDC part way through a reply -- sometimes after zero
        bytes, sometimes mid-line -- while the CAN side stays healthy and the
        board answers normally again seconds later. Left to propagate, the
        AdapterError escaped sweep() entirely and ended a run holding thousands
        of rows, reported as "init failed".

        Recovery is a full initialise(): the board is in an unknown state, and
        ATZ is how this tool reaches a known one. That resets the board's header
        to 0x7DF and its ATST to the firmware default, so initialise()'s own
        forget_header() and the deadline restored here are both load-bearing --
        without them the next request would go out on the functional address,
        which the firmware accepts a reply from ANY responder on.

        The stall is then reported as an ordinary ADAPTER_ERROR reply, so it
        flows into the existing retry and abort logic untouched: a board that
        stalls on everything still trips MAX_CONSECUTIVE_ERRORS and stops the
        run, rather than being retried forever.
        """
        nonlocal deadline_ms
        try:
            reply, _ = adapter.send_did(ecu, did)
            return reply
        except AdapterError as exc:
            print(f"  {ecu}: adapter stalled ({exc}); re-initialising",
                  file=sys.stderr, flush=True)
            # If this raises, the board is not coming back and the run ends --
            # which is correct. A stall we cannot recover from is not transient.
            initialise(adapter, deadline_ms)
            return Reply(ADAPTER_ERROR, raw=f"stalled: {exc}")

    for info in ecus:
        consecutive_silence = 0
        for index, did in enumerate(dids):
            if (info.ecu, did) in skip:
                continue

            reply = probe(info.ecu, did)
            counts["probes"] += 1

            attempt = 1
            while (reply.status == ADAPTER_ERROR
                   and attempt < ADAPTER_ERROR_ATTEMPTS):
                # Re-probing is not cosmetic. classify()'s short-middle-line
                # guard means a REAL multi-frame hit that lost one byte on the
                # wire comes back ADAPTER_ERROR, and within a single run there
                # is no other chance to recover it: an ADAPTER_ERROR row
                # carries no finding, so it would appear in neither the CSV
                # rows nor the session-gated block. already_scanned()
                # separately excludes ADAPTER_ERROR rows from the --resume skip
                # set, so a later run still gets another chance at it -- but
                # that is a whole extra invocation away, and on this car an
                # extra invocation meant re-running discovery too.
                #
                # Spaced by ADAPTER_RETRY_PAUSE_S rather than issued back to
                # back: the faults seen on the car came in bursts across
                # consecutive DIDs, so immediate retries simply re-observe the
                # same burst.
                time.sleep(ADAPTER_RETRY_PAUSE_S)
                reply = probe(info.ecu, did)
                counts["probes"] += 1
                attempt += 1

            # An error that survived the retries gets one read at the generous
            # deadline before it is believed, for the same reason a silence
            # does: it may not be an adapter fault at all, but a reply too
            # large to finish inside the sweep window.
            #
            # session.cpp gives ONE deadline to a whole multi-frame
            # reassembly, and when that deadline lands mid-transfer this car's
            # firmware answers CAN ERROR rather than NO DATA -- so the same
            # oversized reply reaches the host as ADAPTER_ERROR here and as
            # NO_RESPONSE below, depending on where the deadline fell. Scanning
            # the same car twice showed both: scan.jsonl recorded 7E4 0101 (62
            # bytes) as a silence, scan2.jsonl recorded it as CAN ERROR. Of the
            # 159 DIDs the second scan lost, 114 were errors of this kind, with
            # a median payload of 35 bytes and a maximum of 235. Rescuing only
            # silence left the larger half of the problem in place.
            #
            # The retries above all ran at the sweep deadline, so however many
            # there were, none of them tested the one thing that distinguishes
            # these two cases. This does.
            rescued = False
            if reply.status == ADAPTER_ERROR:
                with generous_timeout(adapter, timeout_ms=timeout_ms,
                                      read_timeout_ms=read_timeout_ms):
                    deadline_ms = max(timeout_ms, read_timeout_ms)
                    try:
                        second = probe(info.ecu, did)
                    finally:
                        deadline_ms = timeout_ms
                counts["probes"] += 1
                # Promoted only on a REAL answer. A generous read that comes
                # back NO_RESPONSE or errors again leaves the row as
                # ADAPTER_ERROR on purpose: already_scanned() keeps
                # ADAPTER_ERROR out of the --resume skip set, so the DID stays
                # eligible for another pass, whereas recording it as a silence
                # would bury it permanently on the strength of one bad read.
                if second.status in (POSITIVE, NEGATIVE):
                    reply = second
                    rescued = True

            if reply.status == ADAPTER_ERROR:
                # Counted once per DID, on the LAST attempt's outcome -- not
                # once per probe. Counting every attempt would make one bad DID
                # reach a threshold written for three, moving the abort without
                # anyone deciding to; counting only the first would let the
                # abort be dodged entirely. A DID that errors on every attempt
                # still counts, so MAX_CONSECUTIVE_ERRORS remains exactly as
                # reachable as it was before the retries existed -- it now takes
                # MAX_CONSECUTIVE_ERRORS DIDs that each failed
                # ADAPTER_ERROR_ATTEMPTS times, spread over seconds rather than
                # milliseconds, which is the point.
                consecutive_errors += 1
                if consecutive_errors >= MAX_CONSECUTIVE_ERRORS:
                    recorder.did(info.ecu, did, reply)
                    raise ScanAborted(
                        f"{MAX_CONSECUTIVE_ERRORS} consecutive adapter errors "
                        f"at {info.ecu}/{did:04X}: {reply.raw!r}")
            else:
                consecutive_errors = 0

            # Silence from an ECU that normally answers misses with an NRC is
            # not a miss -- it is the one shape a too-large reply takes.
            # session.cpp gives ONE deadline to a whole multi-frame
            # reassembly, so a payload that cannot finish inside the sweep
            # window comes back NO DATA, indistinguishable from an ECU that
            # said nothing. Phase 4b below only re-reads rows that already came
            # back POSITIVE, which means the biggest payloads on the car -- the
            # ones worth scanning for -- were the only ones it could never
            # rescue. 7E4 0101, the BMS PID the Android app polls, was recorded
            # NO_RESPONSE by a full scan and read back as 62 bytes at
            # --timeout 500.
            #
            # Keyed on miss_mode, and that is a cost decision, not a
            # stylistic one: an ECU calibrated MISS_TIMEOUT answers EVERY
            # unsupported DID with silence, so re-reading each one would add
            # 1536 x --read-timeout -- about thirteen minutes per ECU -- to
            # confirm what calibration already told us. For a MISS_NRC ECU
            # silence is abnormal and rare, so the second look is nearly free.
            if reply.status == NO_RESPONSE and info.miss_mode == MISS_NRC:
                with generous_timeout(adapter, timeout_ms=timeout_ms,
                                      read_timeout_ms=read_timeout_ms):
                    deadline_ms = max(timeout_ms, read_timeout_ms)
                    try:
                        second = probe(info.ecu, did)
                    finally:
                        deadline_ms = timeout_ms
                counts["probes"] += 1
                if second.status != NO_RESPONSE:
                    # Anything but silence is more informative than the silence
                    # it replaces -- a payload, or an NRC naming why not.
                    reply = second
                    rescued = True

            if reply.status == NO_RESPONSE:
                consecutive_silence += 1
                if consecutive_silence >= SLEEP_SUSPICION_PROBES:
                    # Silence alone cannot mean the car left: an ECU whose
                    # calibrated miss_mode is MISS_TIMEOUT answers EVERY
                    # unsupported DID with silence, and unsupported DIDs are
                    # the overwhelming majority of a sweep -- that is normal,
                    # sparse behaviour, not a sign of anything. Before
                    # concluding the car is asleep, re-probe something this
                    # ECU is KNOWN to answer: its discovery DID if discovery
                    # ever saw a POSITIVE from it. CALIBRATION_DID must NEVER
                    # be the fallback here -- it was chosen for calibrate()
                    # precisely because it is unsupported, so for a
                    # MISS_TIMEOUT ecu (the exact case this check exists to
                    # rescue) it is *guaranteed* silent, which just
                    # reproduces the false abort. When no live_did was ever
                    # recorded -- discovery only ever saw a NEGATIVE, or this
                    # ECU came from --ecu and skipped discovery entirely --
                    # fall back to the same identification DIDs discovery
                    # itself tries (F190 then F100), and accept a NEGATIVE
                    # from either as proof of life too: an NRC answer proves
                    # the ECU is listening every bit as much as a POSITIVE
                    # does, which is exactly why discover_ecus() treats a
                    # NEGATIVE as "responds" in the first place.
                    known_good = info.live_did is not None
                    candidates = ([info.live_did] if known_good
                                  else list(DISCOVERY_DIDS))
                    alive = False
                    probed: list[int] = []
                    # At the generous deadline, for the same reason discovery
                    # runs there: the candidates here are identification DIDs,
                    # F190 is a multi-frame VIN, and session.cpp gives the whole
                    # reassembly one deadline. A truncated F190 at the sweep
                    # timeout reads as silence, F100 gets tried, and if that is
                    # silent too a LIVE ECU aborts the run -- the exact bug
                    # class this liveness check was added to fix.
                    with generous_timeout(adapter, timeout_ms=timeout_ms,
                                          read_timeout_ms=read_timeout_ms):
                        deadline_ms = max(timeout_ms, read_timeout_ms)
                        for candidate in candidates:
                            liveness = probe(info.ecu, candidate)
                            counts["probes"] += 1
                            probed.append(candidate)
                            if liveness.status in (POSITIVE, NEGATIVE):
                                alive = True
                                break
                            # NO_RESPONSE: this candidate is silent too, try
                            # the next one. ADAPTER_ERROR: a garbled reply is
                            # not proof the ECU is there, but it is not the
                            # silence this check is looking for either -- a
                            # distressed adapter is MAX_CONSECUTIVE_ERRORS's
                            # job, not this one's -- so it neither confirms
                            # life nor counts against it; just move on to the
                            # next candidate.
                    if alive:
                        consecutive_silence = 0
                    else:
                        probed_text = ", ".join(f"{d:04X}" for d in probed)
                        if known_good:
                            detail = (f"a liveness check against "
                                      f"{probed_text}, a DID it is known "
                                      f"to answer")
                        else:
                            detail = (f"liveness checks against its "
                                      f"identification DIDs ({probed_text})")
                        raise ScanAborted(
                            f"{info.ecu} went silent for "
                            f"{SLEEP_SUSPICION_PROBES} probes, including "
                            f"{detail}. Car asleep? Nothing after this "
                            f"would be a finding. Re-run with --resume "
                            f"once it is awake.")
            else:
                consecutive_silence = 0

            unstable = False
            unconfirmed = False
            if reply.status == POSITIVE and not rescued:
                # Phase 4b: re-read at a generous deadline. Hits are rare, so
                # this costs almost nothing, and it means no recorded payload
                # was ever subject to the fast sweep timeout.
                #
                # Skipped for a rescued row: the read that rescued it was
                # already made at the generous deadline, so it carries the
                # guarantee this phase exists to provide. Re-reading would be a
                # third probe of the same DID for nothing.
                with generous_timeout(adapter, timeout_ms=timeout_ms,
                                      read_timeout_ms=read_timeout_ms):
                    deadline_ms = max(timeout_ms, read_timeout_ms)
                    try:
                        confirmed = probe(info.ecu, did)
                    finally:
                        deadline_ms = timeout_ms
                if confirmed.status == POSITIVE:
                    unstable = (confirmed.payload != reply.payload)
                    reply = confirmed
                else:
                    # The generous re-read did NOT come back POSITIVE --
                    # NO_RESPONSE, NEGATIVE, or ADAPTER_ERROR. Keep the
                    # original fast-sweep hit rather than discard a real
                    # finding, but this row's payload was never actually
                    # confirmed at the generous timeout, which is the
                    # guarantee Phase 4b exists to provide. Flag it as such
                    # -- distinct from "unstable", which means the payload
                    # changed between reads, a different and more alarming
                    # fact than "re-confirmation did not complete".
                    unconfirmed = True

            counts[reply.status.lower()] = counts.get(reply.status.lower(), 0) + 1
            if reply.nrc in (0x7E, 0x7F):
                counts["session_gated"] += 1
            if unstable:
                counts["unstable"] += 1
            if unconfirmed:
                counts["unconfirmed"] += 1

            recorder.did(info.ecu, did, reply, unstable=unstable,
                         unconfirmed=unconfirmed)

            if index % 250 == 0:
                done = counts["probes"]
                rate = done / max(time.perf_counter() - started, 0.001)
                remaining = (len(dids) * len(ecus) - done) / max(rate, 0.001)
                print(f"  {info.ecu} {did:04X}  "
                      f"{counts['positive']} hits  "
                      f"eta {format_eta(remaining)}", flush=True)

    return counts


def summarise(path: str, out) -> int:
    """Render a scan log as CSV of what was found.

    The column order matches the Torque Pro CSVs referenced in the project
    README, so a scan can be diffed against a community PID list directly.
    """
    rows: list[tuple[str, str, int, str]] = []
    gated: list[tuple[str, str]] = []
    try:
        handle = open(path, encoding="utf-8")
    except OSError as exc:
        print(f"cannot read {path}: {exc}", file=sys.stderr)
        return 1
    with handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            # Valid JSON is not necessarily an object: "42", "null", "[1,2,3]"
            # and "true" all parse, and .get() on any of them raises
            # AttributeError. --summary is what an operator reaches for when a
            # run went wrong, so a log with one odd line must still render.
            if not isinstance(record, dict):
                continue
            if record.get("type") != "did":
                continue
            ecu = record.get("ecu")
            did = record.get("did")
            if ecu is None or did is None:
                continue
            if record.get("status") == POSITIVE:
                # unstable and unconfirmed are mutually exclusive at the
                # source (sweep()'s Phase 4b): "unstable" means the payload
                # CHANGED between the fast-sweep hit and the confirming
                # re-read, while "unconfirmed" means that re-read never came
                # back POSITIVE at all, so no confirming read ever happened.
                # These are different kinds of doubt about the row -- a
                # reader diffing against a community PID list needs to see
                # which one applies, not have both collapse into a
                # same-looking blank note.
                note = ("unstable" if record.get("unstable")
                        else "unconfirmed" if record.get("unconfirmed") else "")
                rows.append((ecu, did, record.get("len", 0), note))
            elif record.get("nrc") in ("7E", "7F"):
                gated.append((ecu, did))

    out.write("header,pid,payload_bytes,note\n")
    for ecu, did, length, note in sorted(rows):
        out.write(f"{ecu},22{did},{length},{note}\n")

    if gated:
        out.write(f"# {len(gated)} session-gated DIDs (exist but need an "
                  f"extended session, which this tool does not enter):\n")
        for ecu, did in sorted(gated):
            out.write(f"# {ecu},22{did}\n")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--port", required=False,
                        help="serial device, e.g. /dev/tty.usbmodem101 "
                             "(find it with: pio device list)")
    parser.add_argument("--out", default="scan.jsonl",
                        help="append-only JSONL output (default: scan.jsonl)")
    parser.add_argument("--timeout", type=int, default=50,
                        help="ATST sweep deadline in ms; this is the per-miss "
                             "cost and misses dominate runtime (default: 50, "
                             f"range: {MIN_TIMEOUT_MS}-{MAX_TIMEOUT_MS})")
    parser.add_argument("--read-timeout", type=int, default=500,
                        help="ATST for discovery, the liveness probe, and "
                             "re-reading hits: generous enough for a long "
                             "multi-frame reply. The deadline actually used is "
                             "max(--timeout, --read-timeout), so a value below "
                             "--timeout has no effect (default: 500, range: "
                             f"{MIN_TIMEOUT_MS}-{MAX_TIMEOUT_MS})")
    parser.add_argument("--range", dest="ranges", action="append",
                        help="DID range, e.g. 0100-02FF; repeatable "
                             f"(default: {' '.join(DEFAULT_RANGES)})")
    parser.add_argument("--full", action="store_true",
                        help="sweep all 65,536 DIDs instead of the default ranges")
    parser.add_argument("--ecu", dest="ecus", action="append",
                        help="probe only this ECU, e.g. 7E4; repeatable "
                             "(default: discover them)")
    parser.add_argument("--resume", action="store_true",
                        help="skip (ecu, did) pairs already in --out")
    parser.add_argument("--summary", metavar="JSONL",
                        help="render an existing scan log as CSV and exit; "
                             "does not touch the car or need --port")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.summary:
        return summarise(args.summary, sys.stdout)
    if not args.port:
        print("--port is required unless --summary is given", file=sys.stderr)
        return 2

    for name, value in (("--timeout", args.timeout),
                        ("--read-timeout", args.read_timeout)):
        if value < MIN_TIMEOUT_MS:
            print(f"{name} below {MIN_TIMEOUT_MS} ms is silently clamped by the "
                  f"firmware; refusing so the reported timeout is the real one.",
                  file=sys.stderr)
            return 2
        if value > MAX_TIMEOUT_MS:
            # Same reason as the floor: at_parser.cpp clamps to kMaxTimeoutMs,
            # so anything above it would have the tool print, and the JSONL
            # record, a deadline the board is not using.
            print(f"{name} above {MAX_TIMEOUT_MS} ms is silently clamped by the "
                  f"firmware; refusing so the reported timeout is the real one.",
                  file=sys.stderr)
            return 2

    try:
        dids = expand_ranges(["0000-FFFF"] if args.full
                             else (args.ranges or DEFAULT_RANGES))
    except RangeError as exc:
        print(f"bad --range: {exc}", file=sys.stderr)
        return 2

    try:
        adapter = Adapter(open_serial(
            args.port,
            read_timeout_s=serial_timeout_s(args.timeout, args.read_timeout)))
    except serial.SerialException as exc:
        print(f"cannot open {args.port}: {exc}\n"
              f"List candidates with: pio device list", file=sys.stderr)
        return 1

    try:
        initialise(adapter, args.timeout)
        print(f"adapter ready; {len(dids)} DIDs per ECU, ATST={args.timeout} ms")

        # Read the log once, before choosing where the ECU list comes from:
        # --resume wants both halves of it, the pairs already probed and the
        # ECUs already discovered.
        skip: set[tuple[str, int]] = set()
        reused: list[EcuInfo] = []
        if args.resume:
            try:
                with open(args.out, encoding="utf-8") as handle:
                    lines = handle.readlines()
            except FileNotFoundError:
                print(f"--resume: {args.out} does not exist yet, starting fresh")
            else:
                skip = already_scanned(lines)
                reused = [EcuInfo(row["ecu"], row["vin"], row["miss_mode"],
                                  row["miss_ms"], live_did=row["live_did"],
                                  calibrated=True)
                          for row in recorded_ecus(lines)]
                print(f"resuming: {len(skip)} pairs already recorded")

        known = {info.ecu: info for info in reused}
        if args.ecus:
            # An explicit narrowing of this run. The log is still where the
            # calibration for those addresses lives, so reuse it where it
            # exists rather than re-measuring what is already written down.
            ecus = [known.get(e.upper()) or EcuInfo(e.upper(), None,
                                                    MISS_TIMEOUT, 0.0)
                    for e in args.ecus]
            print(f"using {len(ecus)} ECU(s) from --ecu")
        elif reused:
            ecus = reused
            print(f"reusing {len(ecus)} ECU(s) from {args.out}; skipping "
                  f"discovery")
        else:
            candidates = discovery_candidates()
            print(f"discovering ECUs on {DISCOVERY_FIRST_ADDRESS:03X}-"
                  f"{DISCOVERY_LAST_ADDRESS:03X} "
                  f"({len(candidates)} addresses, "
                  f"{FUNCTIONAL_ADDRESS:03X} excluded)...")
            ecus = discover_ecus(adapter, candidates,
                                 timeout_ms=args.timeout,
                                 read_timeout_ms=args.read_timeout)
        if not ecus:
            print("no ECUs responded. Is the car awake and in Ready mode?",
                  file=sys.stderr)
            return 1

        ecus = [info if info.calibrated
                else calibrate(adapter, info, args.timeout)
                for info in ecus]
        for info in ecus:
            print(f"  {info.ecu}: miss={info.miss_mode} ({info.miss_ms:.1f} ms)")
        if all(info.miss_mode == MISS_TIMEOUT for info in ecus):
            print("warning: no NRC seen from any ECU. If the board is running "
                  "the shipping build rather than rejsacan_usbscan, every "
                  "negative will look like a timeout.", file=sys.stderr)

        total = sum(len(dids) * max(info.miss_ms, 1.0) / 1000.0 for info in ecus)
        print(f"estimated sweep time: {format_eta(total)} "
              f"({len(ecus)} ECUs x {len(dids)} DIDs)")

        recorder = Recorder(args.out)
        recorder.run(firmware=BANNER, timeout_ms=args.timeout,
                     read_timeout_ms=args.read_timeout,
                     ranges=["0000-FFFF"] if args.full
                            else (args.ranges or DEFAULT_RANGES),
                     full=args.full)
        for info in ecus:
            recorder.ecu(info)

        failed = False
        try:
            counts = sweep(adapter, ecus, dids, recorder,
                           timeout_ms=args.timeout,
                           read_timeout_ms=args.read_timeout, skip=skip)
        except (ScanAborted, KeyboardInterrupt) as exc:
            counts = {"aborted": str(exc) or "interrupted"}
            print(f"\nstopped: {counts['aborted']}", file=sys.stderr)
            print(f"re-run with --resume to continue", file=sys.stderr)
        except AdapterError as exc:
            # A stall sweep() could not recover from. Caught HERE rather than
            # left to the outer handler, which would call it "init failed" --
            # the wrong diagnosis for a board that initialised fine and then
            # stopped answering thousands of rows later, and one that sends the
            # operator to the firmware and the port instead of to --resume. The
            # log is intact and resumable; say so, and close it properly.
            counts = {"aborted": f"adapter stalled: {exc}"}
            print(f"\nstopped: {counts['aborted']}", file=sys.stderr)
            print(f"re-run with --resume to continue", file=sys.stderr)
            failed = True
        recorder.end(counts)
        recorder.close()
        print(f"wrote {args.out}: {counts}")
        if failed:
            # Unlike ScanAborted, which is a deliberate stop with the data
            # intact, an adapter that would not come back is a failure of the
            # run and has to be visible as one to whatever invoked it.
            return 1
    except AdapterError as exc:
        print(f"init failed: {exc}", file=sys.stderr)
        return 1
    finally:
        adapter.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
