"""Pure logic for the read-only DID scanner. No I/O, so all of it is testable
on the host.

SAFETY: nothing in this module builds a request. Request construction lives at
the single send_did() chokepoint in did_scan.py, which enforces that the only
UDS service ever sent is 22 (ReadDataByIdentifier). See the project README.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from typing import Iterable

# Where HKMC Data Identifiers actually cluster. A default run sweeps these
# rather than all 65,536, which turns a whole-car pass from an afternoon into a
# couple of minutes while still finding essentially everything. Use --full for
# the exhaustive sweep.
DEFAULT_RANGES: list[str] = [
    "0100-02FF",
    "B000-B0FF",
    "C000-C0FF",
    "E000-E0FF",
    "F100-F1FF",
]


class RangeError(ValueError):
    """A --range specification that cannot be parsed or makes no sense."""


def _parse_did(text: str) -> int:
    stripped = text.strip()
    if not stripped:
        raise RangeError("empty DID specification")
    try:
        value = int(stripped, 16)
    except ValueError as exc:
        raise RangeError(f"{stripped!r} is not hexadecimal") from exc
    if not 0x0000 <= value <= 0xFFFF:
        raise RangeError(f"{stripped!r} is outside 0000-FFFF")
    return value


def expand_ranges(specs: list[str]) -> list[int]:
    """Expand ["0100-02FF", "F190"] into a de-duplicated, ordered DID list.

    Order follows the specs as given; duplicates keep their first position.
    """
    out: list[int] = []
    seen: set[int] = set()
    for spec in specs:
        if "-" in spec:
            low_text, _, high_text = spec.partition("-")
            low, high = _parse_did(low_text), _parse_did(high_text)
            if low > high:
                raise RangeError(f"{spec!r} runs backwards")
            candidates = range(low, high + 1)
        else:
            candidates = [_parse_did(spec)]
        for did in candidates:
            if did not in seen:
                seen.add(did)
                out.append(did)
    return out


POSITIVE = "POSITIVE"
NEGATIVE = "NEGATIVE"
NO_RESPONSE = "NO_RESPONSE"
ADAPTER_ERROR = "ADAPTER_ERROR"

# Fault words the adapter can return. NO DATA means nothing arrived inside the
# ATST window; the rest mean the adapter itself is unhappy.
_NO_RESPONSE_FAULTS = {"NO DATA"}
_ERROR_FAULTS = {"CAN ERROR", "?", "BUFFER FULL", "UNABLE TO CONNECT", "STOPPED"}

_NRC_RE = re.compile(r"^NRC\s+([0-9A-F]{2})$", re.IGNORECASE)
_INDEXED_LINE_RE = re.compile(r"^[0-9A-F]:(.*)$", re.IGNORECASE)
_LENGTH_LINE_RE = re.compile(r"^([0-9A-F]{3,4})$", re.IGNORECASE)

# formatter.cpp's kBytesPerLine: every indexed line carries exactly this many
# bytes except the final one, which holds whatever remains.
_BYTES_PER_LINE = 7


@dataclass
class Reply:
    """One classified adapter reply."""

    status: str
    payload: bytes | None = None
    nrc: int | None = None
    raw: str = ""


def _hex_to_bytes(text: str) -> bytes | None:
    compact = text.replace(" ", "")
    if not compact or len(compact) % 2:
        return None
    try:
        return bytes.fromhex(compact)
    except ValueError:
        return None


def classify(reply: str) -> Reply:
    """Parse one adapter reply.

    The wire format is defined by esp32-s3/src/elm327/formatter.cpp and shared
    with android/.../ResponseParser.kt: payloads of up to 7 bytes are a single
    hex line with no length header; longer ones are a %03X total-length line
    followed by CR-separated lines each prefixed "N:". The index wraps at 0xF,
    so line order — never the prefix — determines byte order.

    "NRC xx" comes from the -DREPORT_NRC scan build. Against the shipping
    firmware a negative response arrives as NO DATA instead, indistinguishable
    from a timeout.
    """
    raw = reply.strip().rstrip(">").strip()
    if not raw:
        return Reply(ADAPTER_ERROR, raw=reply)

    lines = [line.strip() for line in raw.split("\r")]
    lines = [line for line in lines if line]
    if not lines:
        return Reply(ADAPTER_ERROR, raw=raw)

    upper = lines[0].upper()
    if upper in _NO_RESPONSE_FAULTS:
        return Reply(NO_RESPONSE, raw=raw)
    if upper in _ERROR_FAULTS:
        return Reply(ADAPTER_ERROR, raw=raw)

    nrc_match = _NRC_RE.match(lines[0])
    if nrc_match:
        return Reply(NEGATIVE, nrc=int(nrc_match.group(1), 16), raw=raw)

    # Multi-frame: a bare length line, then indexed lines. Matched regardless
    # of how many lines follow: a length line with no data lines behind it is
    # a truncated reply, not a payload, and falls through to the short-read
    # check below via an empty body.
    length_match = _LENGTH_LINE_RE.match(lines[0])
    if length_match:
        declared = int(length_match.group(1), 16)
        data_lines = lines[1:]
        body = bytearray()
        for i, line in enumerate(data_lines):
            indexed = _INDEXED_LINE_RE.match(line)
            if not indexed:
                return Reply(ADAPTER_ERROR, raw=raw)
            chunk = _hex_to_bytes(indexed.group(1))
            if chunk is None:
                return Reply(ADAPTER_ERROR, raw=raw)
            # Every line but the last must carry a full 7 bytes. A short
            # middle line means a byte was dropped on the wire; if we kept
            # concatenating anyway, every byte after it would be silently
            # shifted into the wrong position while still summing to the
            # declared length — a wrong payload recorded as good data,
            # rather than a loud parse failure.
            is_last = i == len(data_lines) - 1
            if not is_last and len(chunk) != _BYTES_PER_LINE:
                return Reply(ADAPTER_ERROR, raw=raw)
            body += chunk
        if len(body) < declared:
            # Short read: the reply was cut off mid-transfer.
            return Reply(ADAPTER_ERROR, raw=raw)
        return Reply(POSITIVE, payload=bytes(body[:declared]), raw=raw)

    # Single frame.
    if len(lines) != 1:
        return Reply(ADAPTER_ERROR, raw=raw)
    payload = _hex_to_bytes(lines[0])
    if payload is None:
        return Reply(ADAPTER_ERROR, raw=raw)
    return Reply(POSITIVE, payload=payload, raw=raw)


def already_scanned(lines: Iterable[str]) -> set[tuple[str, int]]:
    """Read (ecu, did) pairs out of an existing JSONL log, for --resume.

    Silently skips anything unparseable. The file this recovers from is by
    definition one an interrupted run was in the middle of writing, so a
    truncated final line is the normal case, not a corruption to report.

    Rows whose status is ADAPTER_ERROR are excluded from the returned set, so
    a resumed run re-probes them rather than skipping them. An ADAPTER_ERROR
    row carries no finding -- it is what classify() returns for a garbled or
    truncated reply -- and classify's short-middle-line guard means a real
    multi-frame hit with one dropped byte classifies as ADAPTER_ERROR. Without
    this exclusion that DID would be buried and never re-probed by any later
    --resume, with nothing preserved to show it was ever missed. --resume is
    therefore deliberately not idempotent: it can widen the set of DIDs probed
    on a later pass over the same log.
    """
    seen: set[tuple[str, int]] = set()
    for record in _records(lines):
        if record.get("type") != "did":
            continue
        if record.get("status") == ADAPTER_ERROR:
            continue
        ecu, did = record.get("ecu"), record.get("did")
        if not isinstance(ecu, str) or not isinstance(did, str):
            continue
        try:
            seen.add((ecu.upper(), int(did, 16)))
        except ValueError:
            continue
    return seen


def recorded_ecus(lines: Iterable[str]) -> list[dict]:
    """Read the ECUs and their calibration out of an existing JSONL log.

    Discovery writes everything it learns -- which addresses answer, each one's
    VIN, its miss mode and miss cost -- as type:"ecu" rows. A --resume can
    therefore rebuild its ECU list from the log rather than re-probing 231
    addresses at the generous read timeout, which is a four-minute round trip
    before a single new DID gets scanned. That cost is what made an aborted run
    expensive: recovering ~100 rows took longer than the rows were worth.

    Every run appends its own ecu rows, so a twice-resumed log holds several
    copies of each address. The LAST row for an ECU wins -- it is the freshest
    calibration -- while the ECU keeps the position of its first appearance, so
    sweep order stays the discovery order rather than shuffling per resume.
    """
    found: dict[str, dict] = {}
    for record in _records(lines):
        if record.get("type") != "ecu":
            continue
        ecu = record.get("ecu")
        miss_mode = record.get("miss_mode")
        if not isinstance(ecu, str) or not isinstance(miss_mode, str):
            continue
        try:
            miss_ms = float(record.get("miss_ms", 0.0))
        except (TypeError, ValueError):
            continue
        live_did = record.get("live_did")
        if isinstance(live_did, str):
            try:
                live_did = int(live_did, 16)
            except ValueError:
                live_did = None
        elif not isinstance(live_did, int):
            live_did = None
        vin = record.get("vin")
        found[ecu.upper()] = {
            "ecu": ecu.upper(),
            "vin": vin if isinstance(vin, str) else None,
            "miss_mode": miss_mode,
            "miss_ms": miss_ms,
            "live_did": live_did,
        }
    return list(found.values())


def _records(lines: Iterable[str]) -> Iterable[dict]:
    """Yield the usable JSON objects from a JSONL log, skipping the rest.

    Shared by already_scanned() and recorded_ecus() because both read the same
    half-written file and both owe it the same tolerance: json.loads succeeding
    does NOT mean a dict came back -- "42", "null" and "[1,2,3]" are all valid
    JSON documents, and .get() on any of them raises AttributeError.
    """
    for line in lines:
        stripped = line.strip()
        if not stripped:
            continue
        try:
            record = json.loads(stripped)
        except json.JSONDecodeError:
            continue
        if isinstance(record, dict):
            yield record
