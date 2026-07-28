"""Host tests for did_scan's transport. Run: python3 -m unittest test_did_scan -v

No hardware. FakeBoard stands in for env:rejsacan_usbscan and encodes replies
exactly as esp32-s3/src/elm327/formatter.cpp does, so these tests exercise the
same wire contract the real board produces and classify() parses.
"""

import contextlib
import io
import json
import os
import tempfile
import time
import unittest
from unittest import mock

import did_scan
from did_scan import Adapter, AdapterError, initialise
from didscan_core import NEGATIVE, NO_RESPONSE, POSITIVE


class FakeBoard:
    """In-process stand-in for the scan firmware.

    Implements the five stream methods Adapter uses. `sent` records every
    command line, so a test can assert on what went to the car — which is how
    the service-22 safety property gets verified rather than assumed.
    """

    def __init__(self, *, banner: str = "ELM327 v1.5", table: dict | None = None,
                 miss: str = "NO DATA", miss_delay_s: float = 0.0):
        self._banner = banner
        # (ecu, did) -> the full response payload, including the 62 xx xx echo.
        self._table = dict(table or {})
        self._miss = miss
        self._miss_delay_s = miss_delay_s
        self._header = "7DF"
        # AdapterState defaults to echo=true (at_parser.h), so a freshly
        # booted or just-reset board echoes every line it receives until
        # something turns echo off.
        self._echo = True
        self._in = bytearray()
        self._out = bytearray()
        self.sent: list[str] = []
        self.closed = False

    # --- the stream surface Adapter depends on ---

    def reset_input_buffer(self) -> None:
        self._out.clear()

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True

    def write(self, data: bytes) -> int:
        self._in += data
        while b"\r" in self._in:
            line, _, rest = bytes(self._in).partition(b"\r")
            self._in = bytearray(rest)
            self._out += self._reply(line.decode("ascii")).encode("ascii")
        return len(data)

    def read(self, size: int = 1) -> bytes:
        if not self._out:
            return b""          # what pyserial returns on timeout
        chunk = bytes(self._out[:size])
        del self._out[:size]
        return chunk

    # --- firmware behaviour ---

    def _reply(self, line: str) -> str:
        self.sent.append(line)
        # session.cpp captures the echo flag as it stood BEFORE the command
        # is applied, so "ATE0" is itself echoed even though it turns echo
        # off for the command after it — and echo applies uniformly to AT
        # and non-AT lines alike, not just AT ones.
        echo = self._echo
        upper = line.upper()
        body = self._at(upper) if upper.startswith("AT") else self._request(upper)
        if echo:
            body = f"{line}\r{body}"
        return body + "\r>"    # handleLine appends CR and the prompt

    def _at(self, upper: str) -> str:
        if upper == "ATZ":
            # ATZ resets AdapterState{} on the real firmware: echo and the
            # header both revert to their defaults (at_parser.h: echo=true,
            # header=0x7DF), regardless of what a prior session set.
            self._echo = True
            self._header = "7DF"
            return self._banner
        if upper.startswith("ATSH"):
            self._header = upper[4:]
            return "OK"
        if upper.startswith("ATST"):
            return "OK"
        if upper == "ATE0":
            self._echo = False
            return "OK"
        if upper == "ATE1":
            self._echo = True
            return "OK"
        if upper in ("ATS0", "ATS1", "ATL0", "ATSP6"):
            return "OK"
        return "?"

    def _request(self, upper: str) -> str:
        if len(upper) != 6 or not upper.startswith("22"):
            return "?"
        payload = self._table.get((self._header, int(upper[2:], 16)))
        if payload is None:
            if self._miss_delay_s:
                time.sleep(self._miss_delay_s)
            return self._miss
        return self._format(payload)

    @staticmethod
    def _format(data: bytes) -> str:
        """The headers-off, ATS0 form from formatter.cpp."""
        if len(data) <= 7:
            return data.hex().upper()
        lines = [f"{len(data):03X}"]
        for offset in range(0, len(data), 7):
            index = (offset // 7) & 0xF          # formatter.cpp masks with 0xF
            lines.append(f"{index:X}:" + data[offset:offset + 7].hex().upper())
        return "\r".join(lines)


class DeadlineBoard(FakeBoard):
    """A board that only completes a reply when ATST is generous enough.

    Models the reason Phase 4b exists, at the level the tool can observe it:
    session.cpp:76-78 sets ONE deadline for the whole receive-and-reassemble
    loop, so a multi-frame reply needing a flow-control round trip either
    finishes inside ATST or the firmware answers NO DATA. A plain FakeBoard
    answers instantly regardless of ATST, which is why nothing caught discovery
    and the liveness probe running at the fast sweep deadline.
    """

    def __init__(self, *, needs_ms: int = 100, **kwargs):
        super().__init__(**kwargs)
        self._needs_ms = needs_ms
        self._timeout_ms = 200          # AdapterState's default (at_parser.h)

    def _at(self, upper: str) -> str:
        if upper.startswith("ATST"):
            # at_parser.cpp: ATST is in 4 ms units.
            self._timeout_ms = int(upper[4:], 16) * 4
        return super()._at(upper)

    def _request(self, upper: str) -> str:
        if self._timeout_ms < self._needs_ms:
            return self._miss           # "NO DATA" -> NO_RESPONSE
        return super()._request(upper)


class TestInitialise(unittest.TestCase):
    def test_sends_the_expected_setup_sequence(self):
        board = FakeBoard()
        initialise(Adapter(board), 50)
        # ATST is in 4 ms units (at_parser.cpp), so 50 ms -> 0x0C.
        self.assertEqual(board.sent, ["ATZ", "ATE0", "ATS0", "ATSP6", "ATST0C"])

    def test_rejects_a_board_running_the_wrong_firmware(self):
        # The shipping BLE build never answers over USB at all, but a generic
        # ELM327 would answer with its own banner. Either way, stop.
        with self.assertRaises(AdapterError):
            initialise(Adapter(FakeBoard(banner="ELM327 v2.1")), 50)

    def test_ate0_is_itself_echoed_but_setup_still_succeeds(self):
        # Regression test: the real firmware's echo flag defaults on
        # (at_parser.h) and is captured BEFORE a command is applied
        # (session.cpp), so ATE0's own reply is "ATE0\rOK", not "OK" — it
        # turns echo off for what follows, but is itself still echoed.
        # Before _exchange() stripped that prefix, send_at("ATE0") returned
        # "ATE0\rOK", which != "OK", so initialise() raised on the very
        # first real board it ever ran against.
        board = FakeBoard()
        initialise(Adapter(board), 50)  # must not raise
        self.assertEqual(board.sent, ["ATZ", "ATE0", "ATS0", "ATSP6", "ATST0C"])


class TestAdapter(unittest.TestCase):
    def test_only_service_22_is_ever_sent(self):
        # THE safety property, asserted rather than assumed: whatever the caller
        # asks for, every non-AT line reaching the car is a service-22 read.
        board = FakeBoard()
        adapter = Adapter(board)
        for did in (0x0101, 0xF190, 0xFFFF):
            adapter.send_did("7E4", did)
        commands = [c for c in board.sent if not c.upper().startswith("AT")]
        self.assertEqual(commands, ["220101", "22F190", "22FFFF"])

    def test_refuses_the_functional_address(self):
        with self.assertRaises(AdapterError):
            Adapter(FakeBoard()).send_did("7DF", 0x0101)

    def test_refuses_an_out_of_range_did(self):
        with self.assertRaises(AdapterError):
            Adapter(FakeBoard()).send_did("7E4", 0x10000)

    def test_a_non_hex_ecu_address_raises_adapter_error_not_value_error(self):
        # send_did is the safety chokepoint, so its error contract has to be
        # uniform: `--ecu 7g4` used to escape as a bare ValueError traceback out
        # of the one function that decides what reaches the car.
        with self.assertRaises(AdapterError):
            Adapter(FakeBoard()).send_did("7g4", 0x0101)
        with self.assertRaises(AdapterError):
            Adapter(FakeBoard()).send_did("", 0x0101)

    def test_caches_the_header_and_reissues_it_on_change(self):
        board = FakeBoard()
        adapter = Adapter(board)
        adapter.send_did("7E4", 0x0101)
        adapter.send_did("7E4", 0x0102)
        adapter.send_did("7E5", 0x0101)
        self.assertEqual([c for c in board.sent if c.startswith("ATSH")],
                         ["ATSH7E4", "ATSH7E5"])

    def test_single_frame_payload_round_trips(self):
        payload = bytes.fromhex("620101AABBCC")
        board = FakeBoard(table={("7E4", 0x0101): payload})
        reply, _ = Adapter(board).send_did("7E4", 0x0101)
        self.assertEqual(reply.status, POSITIVE)
        self.assertEqual(reply.payload, payload)

    def test_multi_frame_payload_round_trips(self):
        # 20 bytes needs a length header and three indexed lines. Encoding here
        # and decoding in classify() proves both halves agree on the format.
        payload = bytes(range(20))
        board = FakeBoard(table={("7E4", 0x0101): payload})
        reply, _ = Adapter(board).send_did("7E4", 0x0101)
        self.assertEqual(reply.status, POSITIVE)
        self.assertEqual(reply.payload, payload)

    def test_long_payload_past_the_index_wrap_round_trips(self):
        # 17 lines, so the "N:" prefix wraps back to 0 — the case where byte
        # order must come from line order, not the prefix.
        payload = bytes((i * 7) & 0xFF for i in range(119))
        board = FakeBoard(table={("7E4", 0x0101): payload})
        reply, _ = Adapter(board).send_did("7E4", 0x0101)
        self.assertEqual(reply.payload, payload)

    def test_a_miss_is_no_response(self):
        reply, _ = Adapter(FakeBoard()).send_did("7E4", 0x0101)
        self.assertEqual(reply.status, NO_RESPONSE)

    def test_a_negative_response_carries_its_nrc(self):
        reply, _ = Adapter(FakeBoard(miss="NRC 31")).send_did("7E4", 0x0101)
        self.assertEqual(reply.status, NEGATIVE)
        self.assertEqual(reply.nrc, 0x31)

    def test_elapsed_time_tracks_a_slow_board(self):
        # What calibrate() reads to tell a prompt NRC from a silent timeout.
        board = FakeBoard(miss_delay_s=0.05)
        _, elapsed_ms = Adapter(board).send_did("7E4", 0x0101)
        self.assertGreaterEqual(elapsed_ms, 40.0)

    def test_a_board_that_never_answers_raises(self):
        class Silent(FakeBoard):
            def read(self, size: int = 1) -> bytes:
                return b""

        with self.assertRaises(AdapterError):
            Adapter(Silent()).send_at("ATZ")

    def test_close_closes_the_stream(self):
        board = FakeBoard()
        Adapter(board).close()
        self.assertTrue(board.closed)

    def test_reinitialising_forgets_the_cached_header(self):
        # ATZ resets the board's own header back to the functional default
        # 7DF (at_parser.h), so a Python-side header cache that survives a
        # re-init (e.g. a recovery path) could skip ATSH and transmit on
        # whatever the board reset to. On 7DF the firmware accepts a reply
        # from ANY ecu, so two responders would interleave into a corrupt
        # reassembly — the safety property send_did's own 7DF guard cannot
        # catch, because the caller here never asked for 7DF.
        board = FakeBoard(table={("7E4", 0x0101): bytes.fromhex("620101AA")})
        adapter = Adapter(board)
        initialise(adapter, 50)
        adapter.send_did("7E4", 0x0101)
        initialise(adapter, 50)  # e.g. a recovery path re-running setup
        adapter.send_did("7E4", 0x0101)
        header_commands = [c for c in board.sent if c.startswith("ATSH")]
        self.assertEqual(header_commands, ["ATSH7E4", "ATSH7E4"])


from did_scan import (
    DISCOVERY_FIRST_ADDRESS,
    DISCOVERY_LAST_ADDRESS,
    FUNCTIONAL_ADDRESS,
    MISS_NRC,
    MISS_TIMEOUT,
    EcuInfo,
    calibrate,
    discover_ecus,
    discovery_candidates,
    format_eta,
)


def discover(adapter, candidates, *, timeout_ms=50, read_timeout_ms=500):
    """discover_ecus with the two ATST values most of these tests don't vary.

    Discovery raises ATST to --read-timeout for its own duration, because F190
    is a multi-frame VIN and session.cpp deadlines the whole reassembly, so both
    values are required arguments. This keeps them out of the tests that are
    about something else.
    """
    return discover_ecus(adapter, candidates, timeout_ms=timeout_ms,
                         read_timeout_ms=read_timeout_ms)


class TestDiscoveryCandidates(unittest.TestCase):
    """What main() actually hands discover_ecus.

    This is the coverage hole that let the range bug ship: every discovery test
    hand-passed its own short list, so nothing ever exercised the construction,
    and a candidate list that could never contain the BMS passed the whole
    suite. These assert the property -- which ECUs are covered -- not the
    arithmetic that produces it.
    """

    # Exactly the modules android/decode/.../Ioniq5Requests.kt already polls.
    POLLED_ECUS = ("7E4", "7E5", "7B3", "7A0", "7C6")

    def test_contains_every_ecu_the_android_app_already_polls(self):
        # 7E4 is the BMS (state of charge, pack voltage, cell temperatures) and
        # 7E5 the ICCU. Both sit in 0x7E0-0x7E7, above 0x7DF, and both were
        # missing from every run while the upper bound was written as
        # range(0x700, FUNCTIONAL_ADDRESS).
        candidates = discovery_candidates()
        for ecu in self.POLLED_ECUS:
            self.assertIn(ecu, candidates)

    def test_excludes_the_functional_address_and_only_it(self):
        # The forbidden address is 0x7DF and nothing else: on 7DF the firmware
        # accepts a frame from ANY ECU, so two responders would interleave into
        # a corrupt reassembly. Its neighbours must still be probed.
        candidates = discovery_candidates()
        self.assertNotIn(f"{FUNCTIONAL_ADDRESS:03X}", candidates)
        self.assertIn("7DE", candidates)
        self.assertIn("7E0", candidates)

    def test_covers_the_whole_block_with_no_duplicates(self):
        candidates = discovery_candidates()
        expected = (DISCOVERY_LAST_ADDRESS - DISCOVERY_FIRST_ADDRESS + 1) - 1
        self.assertEqual(len(candidates), expected)
        self.assertEqual(len(set(candidates)), len(candidates))

    def test_main_probes_the_candidates_this_function_builds(self):
        # The property above is only worth anything if main() uses it, and the
        # pre-fix code built its own list inline. Drive main() with a FakeBoard
        # and capture what discovery is actually asked to probe.
        seen: list[list[str]] = []

        def fake_discover(adapter, candidates, **kwargs):
            seen.append(list(candidates))
            return []                       # -> "no ECUs responded", exit 1

        board = FakeBoard()
        with contextlib.ExitStack() as stack:
            # open_serial is the ONLY place pyserial is instantiated, which is
            # exactly what makes main() drivable without a port.
            stack.enter_context(mock.patch.object(
                did_scan, "open_serial", lambda *a, **k: board))
            stack.enter_context(mock.patch.object(
                did_scan, "discover_ecus", fake_discover))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            stack.enter_context(contextlib.redirect_stderr(io.StringIO()))
            status = did_scan.main(["--port", "/dev/null"])

        self.assertEqual(status, 1)
        self.assertEqual(seen, [discovery_candidates()])
        for ecu in self.POLLED_ECUS:
            self.assertIn(ecu, seen[0])
        self.assertNotIn("7DF", seen[0])


class TestDiscovery(unittest.TestCase):
    def test_finds_only_responding_addresses_and_decodes_the_vin(self):
        board = FakeBoard(table={
            ("7E4", 0xF190): b"\x62\xF1\x90" + b"KMHKR81FBPU000001",
        })
        found = discover(Adapter(board), ["7E4", "7E5"])
        self.assertEqual([i.ecu for i in found], ["7E4"])
        self.assertEqual(found[0].vin, "KMHKR81FBPU000001")

    def test_falls_back_to_f100_when_f190_is_absent(self):
        board = FakeBoard(table={("7E5", 0xF100): bytes.fromhex("62F100AA")})
        found = discover(Adapter(board), ["7E5"])
        self.assertEqual([i.ecu for i in found], ["7E5"])
        self.assertIsNone(found[0].vin)

    def test_a_negative_response_still_proves_something_is_listening(self):
        found = discover(Adapter(FakeBoard(miss="NRC 31")), ["7E4"])
        self.assertEqual([i.ecu for i in found], ["7E4"])
        self.assertEqual(found[0].miss_mode, MISS_NRC)

    def test_a_silent_bus_finds_nothing(self):
        self.assertEqual(discover(Adapter(FakeBoard()), ["7E4", "7E5"]), [])

    def test_a_short_or_non_alnum_vin_is_not_reported(self):
        # Guards against recording a padded or partial identification block as
        # though it were a VIN.
        board = FakeBoard(table={("7E4", 0xF190): b"\x62\xF1\x90" + b"SHORT"})
        found = discover(Adapter(board), ["7E4"])
        self.assertIsNone(found[0].vin)

    def test_the_f100_fallback_fires_after_an_nrc_not_only_after_silence(self):
        # Finding 1 regression: a NEGATIVE reply to F190 is exactly how a
        # module says "not this DID" -- the fallback exists for precisely
        # this case, not only for silence, so discovery must still try F100
        # rather than stopping at the first NRC.
        board = FakeBoard(table={("7E4", 0xF100): bytes.fromhex("62F100AA")},
                          miss="NRC 31")
        found = discover(Adapter(board), ["7E4"])
        self.assertEqual([i.ecu for i in found], ["7E4"])
        self.assertEqual([c for c in board.sent if c.startswith("22")],
                         ["22F190", "22F100"])

    def test_a_vin_too_slow_for_the_sweep_deadline_is_still_found(self):
        # The whole ECU used to be dropped here. F190 is a ~20-byte reply, so
        # multi-frame, so it needs a flow-control round trip inside the single
        # deadline session.cpp sets for the entire reassembly. Discovery ran at
        # --timeout (50 ms by default, and the docs recommend 20 for long runs);
        # a reply that needed longer came back NO DATA, the host saw
        # NO_RESPONSE, and 7E4 -- the BMS -- simply was not in the run, with no
        # diagnostic anywhere. Discovery must therefore probe at --read-timeout.
        board = DeadlineBoard(needs_ms=100, table={
            ("7E4", 0xF190): b"\x62\xF1\x90" + b"KMHKR81FBPU000001",
        })
        found = discover(Adapter(board), ["7E4"],
                         timeout_ms=50, read_timeout_ms=500)
        self.assertEqual([i.ecu for i in found], ["7E4"])
        self.assertEqual(found[0].vin, "KMHKR81FBPU000001")
        # Raised to 500 ms -> ATST7D, and the sweep value put back afterwards.
        self.assertIn("ATST7D", board.sent)
        self.assertEqual(board.sent[-1], "ATST0C")

    def test_the_sweep_deadline_is_restored_even_when_nothing_is_found(self):
        # The restore has to be unconditional: whatever runs next (calibrate,
        # then the sweep) budgets its per-miss cost on --timeout, so leaving the
        # board on the generous value would silently multiply the sweep's
        # runtime by ten.
        board = FakeBoard()
        self.assertEqual(discover(Adapter(board), ["7E4"]), [])
        self.assertEqual(board.sent[-1], "ATST0C")


class TestCalibrate(unittest.TestCase):
    def test_a_prompt_nrc_is_the_fast_miss_mode(self):
        info = calibrate(Adapter(FakeBoard(miss="NRC 31")),
                         EcuInfo("7E4", None, MISS_TIMEOUT, 0.0), 50)
        self.assertEqual(info.miss_mode, MISS_NRC)

    def test_silence_is_the_slow_miss_mode_and_gets_timed(self):
        info = calibrate(Adapter(FakeBoard(miss_delay_s=0.03)),
                         EcuInfo("7E4", None, MISS_NRC, 0.0), 50)
        self.assertEqual(info.miss_mode, MISS_TIMEOUT)
        self.assertGreaterEqual(info.miss_ms, 20.0)

    def test_the_vin_survives_calibration(self):
        info = calibrate(Adapter(FakeBoard()),
                         EcuInfo("7E4", "KMHKR81FBPU000001", MISS_NRC, 0.0),
                         50)
        self.assertEqual(info.vin, "KMHKR81FBPU000001")

    def test_a_positive_reply_to_ffff_gets_the_pessimistic_cost(self):
        # Finding 2 regression: FFFF answering at all is surprising and
        # breaks the "certain to be unsupported" assumption CALIBRATION_DID
        # relies on. The elapsed time of that one prompt reply is not a
        # genuine miss cost, so the recorded cost must be the pessimistic
        # ATST window, not the (fast) measured elapsed_ms -- which would
        # otherwise underestimate the sweep, and an underestimated ETA is
        # worse than none, because the operator plans a session around it.
        board = FakeBoard(table={("7E4", 0xFFFF): bytes.fromhex("62FFFFAA")})
        info = calibrate(Adapter(board), EcuInfo("7E4", None, MISS_NRC, 0.0),
                         50)
        self.assertEqual(info.miss_mode, MISS_TIMEOUT)
        self.assertEqual(info.miss_ms, 50.0)

    def test_an_adapter_error_on_ffff_also_gets_the_pessimistic_cost(self):
        # Same reasoning as above for a garbled reply: ADAPTER_ERROR must not
        # be lumped silently into "must be fine" alongside POSITIVE, and must
        # not be charged its own (fast) elapsed time either.
        board = FakeBoard(miss="CAN ERROR")
        info = calibrate(Adapter(board), EcuInfo("7E4", None, MISS_NRC, 0.0),
                         50)
        self.assertEqual(info.miss_mode, MISS_TIMEOUT)
        self.assertEqual(info.miss_ms, 50.0)


class TestFormatEta(unittest.TestCase):
    def test_scales_the_unit_to_the_magnitude(self):
        self.assertEqual(format_eta(9), "9s")
        self.assertEqual(format_eta(600), "10m")
        self.assertEqual(format_eta(7200), "2.0h")


from did_scan import (
    ADAPTER_ERROR_ATTEMPTS,
    ADAPTER_RETRY_PAUSE_S,
    MAX_CONSECUTIVE_ERRORS,
    SLEEP_SUSPICION_PROBES,
    Recorder,
    ScanAborted,
    summarise,
    sweep,
)


class SweepTestCase(unittest.TestCase):
    """Shared scaffolding: a throwaway JSONL path and a reader for its rows."""

    def setUp(self):
        handle = tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False)
        handle.close()
        self.path = handle.name
        # sweep() spaces its retries with a real pause, which every test that
        # drives a CAN ERROR would otherwise pay. The pause itself is pinned by
        # test_the_retries_are_spaced_so_a_transient_fault_can_clear; here it is
        # only noise, so record the calls instead of sleeping through them.
        patcher = mock.patch.object(did_scan.time, "sleep")
        self.slept = patcher.start()
        self.addCleanup(patcher.stop)

    def tearDown(self):
        os.unlink(self.path)

    def records(self, kind: str | None = None) -> list[dict]:
        with open(self.path, encoding="utf-8") as handle:
            rows = [json.loads(line) for line in handle if line.strip()]
        return [r for r in rows if kind is None or r["type"] == kind]

    def run_sweep(self, board, dids, *, skip=None, ecus=None):
        recorder = Recorder(self.path)
        try:
            return sweep(Adapter(board),
                         ecus or [EcuInfo("7E4", None, MISS_NRC, 1.0)],
                         dids, recorder, timeout_ms=50, read_timeout_ms=500,
                         skip=skip or set())
        finally:
            recorder.close()


class TestSweep(SweepTestCase):
    def test_records_one_row_per_did_with_the_right_statuses(self):
        board = FakeBoard(table={("7E4", 0x0101): bytes.fromhex("620101AA")})
        counts = self.run_sweep(board, [0x0101, 0x0102])
        self.assertEqual([r["did"] for r in self.records("did")], ["0101", "0102"])
        self.assertEqual(counts["positive"], 1)
        self.assertEqual(counts["no_response"], 1)
        # Three probes for two DIDs: the silent one is re-read once at the
        # generous deadline before its miss is believed, because this ECU is
        # calibrated MISS_NRC and silence from it is abnormal.
        self.assertEqual(counts["probes"], 3)

    def test_a_hit_records_its_payload_and_length(self):
        payload = bytes.fromhex("620101AABBCC")
        board = FakeBoard(table={("7E4", 0x0101): payload})
        self.run_sweep(board, [0x0101])
        row = self.records("did")[0]
        self.assertEqual(row["payload"], payload.hex().upper())
        self.assertEqual(row["len"], len(payload))

    def test_a_hit_is_re_read_at_the_generous_timeout_and_the_sweep_value_restored(self):
        # Phase 4b. 500 ms -> ATST7D, then back to 50 ms -> ATST0C.
        board = FakeBoard(table={("7E4", 0x0101): bytes.fromhex("620101AA")})
        self.run_sweep(board, [0x0101])
        self.assertIn("ATST7D", board.sent)
        self.assertEqual(board.sent.count("220101"), 2)
        self.assertEqual(board.sent[-1], "ATST0C")

    def test_a_negative_is_not_re_read(self):
        # An NRC is the ECU stating this DID is unsupported. That answer
        # arrived intact and needs no confirming; only silence is ambiguous.
        board = FakeBoard(miss="NRC 31")
        self.run_sweep(board, [0x0101])
        self.assertEqual(board.sent.count("220101"), 1)
        self.assertNotIn("ATST7D", board.sent)

    def test_an_error_that_clears_at_the_generous_deadline_is_recorded_as_the_hit(self):
        # The OTHER shape a too-large reply takes, and the one that matters
        # most on this car. A multi-frame reply that cannot finish inside the
        # 50 ms sweep window does not necessarily come back as silence: the
        # firmware reports CAN ERROR, which classify() maps to ADAPTER_ERROR.
        #
        # Measured by scanning the same car twice. scan.jsonl recorded 7E4 0101
        # (62 bytes) as NO_RESPONSE; scan2.jsonl, run against the corrected
        # firmware, recorded the very same DID as ADAPTER_ERROR 'CAN ERROR'.
        # 114 of 159 DIDs confirmed by the first scan and lost by the second
        # were errors of exactly this kind, with a median payload of 35 bytes
        # and a maximum of 235 -- the biggest and most valuable rows in the
        # inventory. Rescuing only silence closed one door and left this one
        # open.
        board = DeadlineBoard(needs_ms=300, miss="CAN ERROR",
                              table={("7E4", 0x0101): bytes.fromhex(
                                  "620101" + "AA" * 59)})
        counts = self.run_sweep(board, [0x0101])
        row = self.records("did")[0]
        self.assertEqual(row["status"], POSITIVE)
        self.assertEqual(row["len"], 62)
        self.assertEqual(counts["adapter_error"], 0)

    def test_a_rescued_error_does_not_count_toward_the_abort(self):
        # If the generous read succeeds, the adapter was never distressed --
        # the deadline was simply too tight. Counting that toward
        # MAX_CONSECUTIVE_ERRORS would abort a healthy run precisely when it
        # found a run of large payloads, which is the opposite of what the
        # abort is for.
        dids = list(range(0x0101, 0x0101 + MAX_CONSECUTIVE_ERRORS))
        table = {("7E4", d): bytes.fromhex("6201" + f"{d & 0xFF:02X}"
                                           + "AA" * 59) for d in dids}
        board = DeadlineBoard(needs_ms=300, miss="CAN ERROR", table=table)
        counts = self.run_sweep(board, dids)      # must not raise
        self.assertEqual(counts["positive"], len(dids))
        self.assertEqual(counts["adapter_error"], 0)

    def test_an_error_that_persists_at_the_generous_deadline_still_aborts(self):
        # The abort must stay reachable: a genuinely distressed adapter errors
        # at every deadline, and nothing here should hide that.
        dids = list(range(0x0100, 0x0100 + MAX_CONSECUTIVE_ERRORS))
        with self.assertRaises(ScanAborted):
            self.run_sweep(FakeBoard(miss="CAN ERROR"), dids)

    def test_the_generous_re_read_of_an_error_happens_once(self):
        # One second opinion, not a second retry loop. The retries already ran
        # at the sweep deadline; this is a single probe at the wider one.
        board = FakeBoard(miss="CAN ERROR")
        self.run_sweep(board, [0x0101])
        self.assertEqual(board.sent.count("220101"),
                         ADAPTER_ERROR_ATTEMPTS + 1)

    def test_a_rescued_error_skips_phase_4b(self):
        # The read that rescued it was already made at the generous deadline,
        # so it carries that phase's guarantee; re-reading would be a fourth
        # probe of the same DID for nothing.
        board = DeadlineBoard(needs_ms=300, miss="CAN ERROR",
                              table={("7E4", 0x0101): bytes.fromhex(
                                  "620101" + "AA" * 59)})
        self.run_sweep(board, [0x0101])
        # ADAPTER_ERROR_ATTEMPTS fast probes, then one generous read.
        self.assertEqual(board.sent.count("220101"),
                         ADAPTER_ERROR_ATTEMPTS + 1)

    def test_silence_from_an_nrc_ecu_is_re_read_at_the_generous_deadline(self):
        # THE FALSE NEGATIVE, measured on the car. session.cpp gives ONE
        # deadline to a whole multi-frame reassembly, so a reply too large to
        # finish inside the 50 ms sweep window comes back NO DATA -- which the
        # host cannot tell from an ECU that said nothing. Phase 4b only ever
        # re-read rows that were already POSITIVE, so exactly the biggest and
        # most valuable payloads were the ones never given a second look.
        #
        # 7E4 0101 -- the BMS PID the Android app polls for SoC, pack voltage
        # and cell temperatures -- was recorded NO_RESPONSE by a full scan and
        # then read back as a 62-byte POSITIVE at --timeout 500.
        #
        # Silence is only suspicious on an ECU calibrated MISS_NRC: that ECU
        # answers unsupported DIDs with an NRC, so silence from it is abnormal
        # and worth one generous re-read.
        board = DeadlineBoard(needs_ms=300,
                              table={("7E4", 0x0101): bytes.fromhex(
                                  "620101" + "AA" * 59)})
        counts = self.run_sweep(board, [0x0101])
        row = self.records("did")[0]
        self.assertEqual(row["status"], POSITIVE)
        self.assertEqual(row["len"], 62)
        self.assertEqual(counts["positive"], 1)
        self.assertEqual(counts["no_response"], 0)

    def test_silence_from_a_timeout_ecu_is_not_re_read(self):
        # THE COST GUARD, and why this is keyed on miss_mode rather than
        # applied to every silence. An ECU calibrated MISS_TIMEOUT answers
        # EVERY unsupported DID with silence, and unsupported DIDs are the
        # overwhelming majority of a sweep. Re-reading each of them at the
        # generous deadline would cost 1536 x 500 ms -- around thirteen
        # minutes per ECU -- to confirm what calibration already established.
        board = DeadlineBoard(needs_ms=300,
                              table={("7B3", 0x0101): bytes.fromhex(
                                  "620101" + "AA" * 59)})
        self.run_sweep(board, [0x0101],
                       ecus=[EcuInfo("7B3", None, MISS_TIMEOUT, 50.0)])
        self.assertEqual(self.records("did")[0]["status"], NO_RESPONSE)
        self.assertEqual(board.sent.count("220101"), 1)

    def test_a_re_read_that_stays_silent_is_still_recorded_as_a_miss(self):
        # The generous re-read is a second opinion, not a promotion: a DID
        # that is genuinely absent must still end up NO_RESPONSE.
        board = FakeBoard()
        self.run_sweep(board, [0x0101])
        self.assertEqual(self.records("did")[0]["status"], NO_RESPONSE)
        self.assertEqual(board.sent.count("220101"), 2)   # sweep + one re-read

    def test_a_payload_that_changes_between_reads_is_flagged_unstable(self):
        class Flaky(FakeBoard):
            def __init__(self):
                super().__init__(table={("7E4", 0x0101): bytes.fromhex("620101AA")})
                self._calls = 0

            def _request(self, upper: str) -> str:
                self._calls += 1
                if self._calls == 2:          # the Phase 4b re-read
                    return self._format(bytes.fromhex("620101AABB"))
                return super()._request(upper)

        counts = self.run_sweep(Flaky(), [0x0101])
        self.assertEqual(counts["unstable"], 1)
        self.assertTrue(self.records("did")[0]["unstable"])

    def test_session_gated_negatives_are_counted_separately(self):
        counts = self.run_sweep(FakeBoard(miss="NRC 7E"), [0x0101, 0x0102])
        self.assertEqual(counts["session_gated"], 2)
        self.assertEqual(counts["negative"], 2)

    def test_a_hit_that_goes_silent_on_re_read_keeps_the_original_payload_and_is_flagged_unconfirmed(self):
        # Finding 1 regression: the fast-sweep hit is a real finding even if
        # the generous re-read cannot confirm it (NO_RESPONSE this time), so
        # it must still be recorded -- but never as though it had been
        # confirmed at the generous timeout, which is the one guarantee
        # Phase 4b exists to provide.
        class GoesSilentOnReread(FakeBoard):
            def __init__(self):
                super().__init__(table={("7E4", 0x0101): bytes.fromhex("620101AA")})
                self._calls = 0

            def _request(self, upper: str) -> str:
                self._calls += 1
                if self._calls == 2:          # the Phase 4b re-read
                    return self._miss          # "NO DATA" -> NO_RESPONSE
                return super()._request(upper)

        counts = self.run_sweep(GoesSilentOnReread(), [0x0101])
        self.assertEqual(counts["unconfirmed"], 1)
        row = self.records("did")[0]
        self.assertEqual(row["status"], POSITIVE)
        self.assertEqual(row["payload"], "620101AA")  # the original hit, kept
        self.assertTrue(row["unconfirmed"])
        self.assertNotIn("unstable", row)

    def test_a_hit_whose_re_read_is_garbled_keeps_the_original_payload_and_is_flagged_unconfirmed(self):
        # Same regression as above, but the re-read comes back ADAPTER_ERROR
        # (a garbled reply) rather than silence -- either non-POSITIVE
        # outcome must be handled the same way.
        class GarbledOnReread(FakeBoard):
            def __init__(self):
                super().__init__(table={("7E4", 0x0101): bytes.fromhex("620101AA")})
                self._calls = 0

            def _request(self, upper: str) -> str:
                self._calls += 1
                if self._calls == 2:          # the Phase 4b re-read
                    return "?"                 # unrecognised -> ADAPTER_ERROR
                return super()._request(upper)

        counts = self.run_sweep(GarbledOnReread(), [0x0101])
        self.assertEqual(counts["unconfirmed"], 1)
        row = self.records("did")[0]
        self.assertEqual(row["status"], POSITIVE)
        self.assertEqual(row["payload"], "620101AA")
        self.assertTrue(row["unconfirmed"])

    def test_the_skip_set_is_honoured(self):
        counts = self.run_sweep(FakeBoard(), [0x0101, 0x0102],
                                skip={("7E4", 0x0101)})
        self.assertEqual([r["did"] for r in self.records("did")], ["0102"])
        # 0101 is skipped entirely; 0102 is probed and, being silent on a
        # MISS_NRC ECU, re-read once at the generous deadline.
        self.assertEqual(counts["probes"], 2)

    def test_three_consecutive_adapter_errors_abort_the_run(self):
        # "SEARCHING..." is not a recognised reply, so classify() calls it an
        # adapter error — a distressed adapter produces junk, not findings.
        with self.assertRaises(ScanAborted):
            self.run_sweep(FakeBoard(miss="SEARCHING..."),
                           list(range(0x0100, 0x0110)))

    def test_one_fewer_than_max_consecutive_errors_does_not_abort(self):
        # Finding 3: a boundary test. The tests above only prove an abort
        # eventually happens over a range far larger than the threshold, so
        # they would pass unnoticed if MAX_CONSECUTIVE_ERRORS were
        # accidentally changed to 1 or 2. One error short of the threshold
        # must complete the sweep normally.
        dids = list(range(0x0100, 0x0100 + MAX_CONSECUTIVE_ERRORS - 1))
        counts = self.run_sweep(FakeBoard(miss="SEARCHING..."), dids)
        self.assertEqual(counts["adapter_error"], MAX_CONSECUTIVE_ERRORS - 1)

    def test_exactly_max_consecutive_errors_aborts(self):
        # The other half of the boundary: right at the threshold must abort.
        dids = list(range(0x0100, 0x0100 + MAX_CONSECUTIVE_ERRORS))
        with self.assertRaises(ScanAborted):
            self.run_sweep(FakeBoard(miss="SEARCHING..."), dids)

    def test_a_long_silence_aborts_rather_than_recording_false_misses(self):
        with self.assertRaises(ScanAborted):
            self.run_sweep(FakeBoard(), list(range(0x0000, 0x0100)))

    def test_one_fewer_than_the_sleep_threshold_does_not_abort(self):
        # Finding 3: the same boundary problem for SLEEP_SUSPICION_PROBES.
        # One silent probe short of the threshold must never trigger the
        # liveness check at all, let alone abort.
        dids = list(range(0x0000, SLEEP_SUSPICION_PROBES - 1))
        counts = self.run_sweep(FakeBoard(), dids)
        self.assertEqual(counts["no_response"], SLEEP_SUSPICION_PROBES - 1)

    def test_exactly_the_sleep_threshold_aborts_when_the_liveness_check_is_also_silent(self):
        # Right at the threshold, with nothing (not even the liveness probe)
        # answering, this really is indistinguishable from a car that left.
        dids = list(range(0x0000, SLEEP_SUSPICION_PROBES))
        with self.assertRaises(ScanAborted):
            self.run_sweep(FakeBoard(), dids)

    def test_a_silent_miss_ecu_sweeps_straight_through_the_threshold(self):
        # Finding 2 regression: an ECU whose calibrated miss_mode is
        # MISS_TIMEOUT answers EVERY unsupported DID with silence, and
        # unsupported DIDs are the overwhelming majority of a sweep -- that
        # is normal, sparse behaviour, not a car going to sleep. Threading
        # the discovery DID through as live_did lets the liveness check
        # prove the ECU is still there, so the sweep must run straight
        # through what used to be an immediate, spurious abort.
        board = FakeBoard(table={("7E4", 0xF190): bytes.fromhex("62F19011")})
        ecu = EcuInfo("7E4", None, MISS_TIMEOUT, 1.0, live_did=0xF190)
        dids = list(range(0x0200, 0x0200 + SLEEP_SUSPICION_PROBES + 50))
        counts = self.run_sweep(board, dids, ecus=[ecu])
        self.assertEqual(counts["no_response"], len(dids))
        self.assertEqual(counts["probes"], len(dids) + 1)  # +1 liveness probe

    def test_a_genuinely_dead_ecu_still_aborts_even_with_a_known_good_did(self):
        # The liveness probe distinguishes "sparse" from "gone" -- it must
        # not disable the sleep detector. If the known-good DID has ALSO
        # gone silent, the abort must still fire.
        ecu = EcuInfo("7E4", None, MISS_TIMEOUT, 1.0, live_did=0xF190)
        dids = list(range(0x0200, 0x0200 + SLEEP_SUSPICION_PROBES))
        with self.assertRaises(ScanAborted):
            self.run_sweep(FakeBoard(), dids, ecus=[ecu])

    def test_a_live_ecu_with_no_known_good_did_sweeps_past_the_threshold(self):
        # Round 2 of Finding 2: live_did is None does NOT imply MISS_NRC.
        # calibrate() derives miss_mode from the 0xFFFF reply alone, so an
        # ECU discovered only via a NEGATIVE (no live_did recorded) can
        # still calibrate as MISS_TIMEOUT -- silent to CALIBRATION_DID and
        # to every other unsupported DID, including the swept range here.
        # Falling back to CALIBRATION_DID for the liveness check would ask
        # the one question already known to be silent and false-abort on a
        # live ECU. The fallback must instead try the identification DIDs
        # (F190 then F100), and an NRC to F190 -- not a POSITIVE -- must
        # still count as proof of life.
        class NrcOnlyForF190(FakeBoard):
            def _request(self, upper: str) -> str:
                if upper == "22F190":
                    return "NRC 31"
                return super()._request(upper)  # NO DATA for everything else

        ecu = EcuInfo("7E4", None, MISS_TIMEOUT, 1.0)   # live_did defaults None
        dids = list(range(0x0200, 0x0200 + SLEEP_SUSPICION_PROBES + 50))
        counts = self.run_sweep(NrcOnlyForF190(), dids, ecus=[ecu])
        self.assertEqual(counts["no_response"], len(dids))
        self.assertEqual(counts["probes"], len(dids) + 1)  # F190 alone answers

    def test_the_same_shape_reached_through_ecu_style_construction_also_sweeps_through(self):
        # Same regression as above, but built exactly the way main() builds
        # EcuInfo for --ecu (did_scan.py: EcuInfo(e, None, MISS_TIMEOUT,
        # 0.0)) -- no discovery, so no live_did, ever. A user who already
        # knows their ECU address takes this path, and it is probably the
        # most common invocation.
        class NrcOnlyForF190(FakeBoard):
            def _request(self, upper: str) -> str:
                if upper == "22F190":
                    return "NRC 31"
                return super()._request(upper)

        ecu = EcuInfo("7E4", None, MISS_TIMEOUT, 0.0)
        dids = list(range(0x0300, 0x0300 + SLEEP_SUSPICION_PROBES + 10))
        counts = self.run_sweep(NrcOnlyForF190(), dids, ecus=[ecu])
        self.assertEqual(counts["no_response"], len(dids))

    def test_a_dead_ecu_with_no_known_good_did_still_aborts(self):
        # The fallback must not become a way to dodge the sleep detector:
        # if F190, F100, and everything else are silent, this really is a
        # dead (or asleep) ECU, and the abort must still fire.
        ecu = EcuInfo("7E4", None, MISS_TIMEOUT, 1.0)
        dids = list(range(0x0400, 0x0400 + SLEEP_SUSPICION_PROBES))
        with self.assertRaises(ScanAborted):
            self.run_sweep(FakeBoard(), dids, ecus=[ecu])

    def test_the_abort_message_does_not_claim_a_did_is_known_answering_when_the_fallback_was_used(self):
        # Regression for the wrong claim in the old message: when the
        # fallback identification DIDs were used (no live_did), the abort
        # must not assert that a DID is "known to answer" -- it was tried,
        # not known.
        ecu = EcuInfo("7E4", None, MISS_TIMEOUT, 1.0)
        dids = list(range(0x0500, 0x0500 + SLEEP_SUSPICION_PROBES))
        with self.assertRaises(ScanAborted) as ctx:
            self.run_sweep(FakeBoard(), dids, ecus=[ecu])
        self.assertNotIn("known to answer", str(ctx.exception))

    def test_an_adapter_error_that_succeeds_on_retry_is_recorded_as_the_hit(self):
        # The spec's "record it, retry once". classify()'s short-middle-line
        # guard means a real multi-frame hit that dropped a byte comes back
        # ADAPTER_ERROR, and --resume keys on (ecu, did) alone -- so without the
        # retry a genuine DID is buried by every later run, in neither the CSV
        # rows nor the gated block.
        class ErrorsOnce(FakeBoard):
            def __init__(self):
                super().__init__(
                    table={("7E4", 0x0101): bytes.fromhex("620101AA")})
                self._calls = 0

            def _request(self, upper: str) -> str:
                self._calls += 1
                if self._calls == 1:
                    return "CAN ERROR"
                return super()._request(upper)

        counts = self.run_sweep(ErrorsOnce(), [0x0101])
        row = self.records("did")[0]
        self.assertEqual(row["status"], POSITIVE)
        self.assertEqual(row["payload"], "620101AA")
        self.assertEqual(counts["adapter_error"], 0)
        self.assertEqual(counts["positive"], 1)

    def test_an_adapter_error_that_errors_on_every_attempt_is_recorded_as_an_error(self):
        # A bounded retry, not a retry loop: a DID that is genuinely unreadable
        # must still produce exactly one row, and that row must say
        # ADAPTER_ERROR.
        board = FakeBoard(miss="CAN ERROR")
        counts = self.run_sweep(board, [0x0101])
        rows = self.records("did")
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["status"], "ADAPTER_ERROR")
        self.assertEqual(counts["adapter_error"], 1)
        # The fast retries, plus the one generous re-read that rules out an
        # oversized reply before the error is believed.
        self.assertEqual(board.sent.count("220101"),
                         ADAPTER_ERROR_ATTEMPTS + 1)

    def test_an_adapter_error_that_clears_on_the_last_attempt_is_recorded_as_the_hit(self):
        # The failure this exists for, observed on a real car: CAN ERROR came
        # in bursts spanning several consecutive DIDs, and the same DIDs
        # answered NRC 31 cleanly when a later --resume re-probed them. A
        # single back-to-back retry was not enough to ride out a burst -- both
        # attempts landed inside it -- so three runs aborted on transient
        # faults that had nothing to do with the DIDs they stopped at.
        class ErrorsUntilLastAttempt(FakeBoard):
            def __init__(self):
                super().__init__(
                    table={("7E4", 0x0101): bytes.fromhex("620101AA")})
                self._calls = 0

            def _request(self, upper: str) -> str:
                self._calls += 1
                if self._calls < ADAPTER_ERROR_ATTEMPTS:
                    return "CAN ERROR"
                return super()._request(upper)

        counts = self.run_sweep(ErrorsUntilLastAttempt(), [0x0101])
        row = self.records("did")[0]
        self.assertEqual(row["status"], POSITIVE)
        self.assertEqual(row["payload"], "620101AA")
        self.assertEqual(counts["adapter_error"], 0)

    def test_the_retries_are_spaced_so_a_transient_fault_can_clear(self):
        # Spacing is the whole point. Two attempts issued back to back land
        # inside the same bus fault and tell you nothing new; the pause is what
        # gives the transceiver time to recover before the verdict is taken.
        self.run_sweep(FakeBoard(miss="CAN ERROR"), [0x0101])
        self.assertEqual(self.slept.call_args_list,
                         [mock.call(ADAPTER_RETRY_PAUSE_S)]
                         * (ADAPTER_ERROR_ATTEMPTS - 1))

    def test_a_clean_probe_never_pauses(self):
        # The pause must cost nothing on the overwhelmingly common path: a
        # sweep is almost entirely misses, and a per-probe sleep would add
        # hours to a full run.
        self.run_sweep(FakeBoard(), [0x0101, 0x0102])
        self.slept.assert_not_called()

    def test_three_erroring_dids_still_abort_despite_the_retry(self):
        # The retry must not move the abort threshold in either direction: the
        # count is per DID, on the retry's outcome, so MAX_CONSECUTIVE_ERRORS
        # erroring DIDs still stop the run -- no easier (which counting both
        # halves would have made it) and no harder (which counting only the
        # first probe would have made it).
        dids = list(range(0x0100, 0x0100 + MAX_CONSECUTIVE_ERRORS))
        with self.assertRaises(ScanAborted):
            self.run_sweep(FakeBoard(miss="CAN ERROR"), dids)

    def test_a_liveness_probe_too_slow_for_the_sweep_deadline_does_not_abort(self):
        # Same bug class as discovery, and the more dangerous half: the liveness
        # candidates are identification DIDs, F190 is a multi-frame VIN, and a
        # truncated F190 reads as silence. F100 is then tried and is also
        # silent, so a LIVE ECU aborts the run -- resurrecting exactly what the
        # liveness check was added to prevent. The probe must be made at
        # --read-timeout.
        board = DeadlineBoard(
            needs_ms=100,
            table={("7E4", 0xF190): b"\x62\xF1\x90" + b"KMHKR81FBPU000001"})
        ecu = EcuInfo("7E4", None, MISS_TIMEOUT, 1.0, live_did=0xF190)
        dids = list(range(0x0200, 0x0200 + SLEEP_SUSPICION_PROBES + 10))
        counts = self.run_sweep(board, dids, ecus=[ecu])
        self.assertEqual(counts["no_response"], len(dids))
        raised_at = board.sent.index("ATST7D")
        # ...and the sweep deadline put back afterwards, or every remaining miss
        # would cost ten times what the ETA was calculated from.
        self.assertIn("ATST0C", board.sent[raised_at:])

    def test_multiple_ecus_are_swept_in_order(self):
        board = FakeBoard(table={("7E5", 0x0101): bytes.fromhex("620101AA")})
        self.run_sweep(board, [0x0101],
                       ecus=[EcuInfo("7E4", None, MISS_NRC, 1.0),
                             EcuInfo("7E5", None, MISS_NRC, 1.0)])
        self.assertEqual([r["ecu"] for r in self.records("did")], ["7E4", "7E5"])


class TestRecorder(SweepTestCase):
    def test_run_and_ecu_and_end_records_carry_their_fields(self):
        recorder = Recorder(self.path)
        recorder.run(firmware="ELM327 v1.5", timeout_ms=50, read_timeout_ms=500,
                     ranges=["0100-0101"], full=False)
        recorder.ecu(EcuInfo("7E4", "KMHKR81FBPU000001", MISS_NRC, 4.8))
        recorder.end({"probes": 2})
        recorder.close()

        run, ecu, end = self.records()
        self.assertEqual(run["timeout_ms"], 50)
        self.assertEqual(run["read_timeout_ms"], 500)
        self.assertEqual(run["ranges"], ["0100-0101"])
        self.assertEqual(ecu["ecu"], "7E4")
        self.assertEqual(ecu["vin"], "KMHKR81FBPU000001")
        self.assertEqual(ecu["miss_ms"], 4.8)
        self.assertEqual(end["probes"], 2)

    def test_the_ecu_row_records_the_live_did_so_a_resume_can_reuse_it(self):
        # sweep()'s liveness check re-probes live_did to tell "asleep" from
        # "sparse". A --resume that rebuilds its ECUs from the log rather than
        # from discovery can only keep that check if the DID was written down.
        recorder = Recorder(self.path)
        recorder.ecu(EcuInfo("7E4", None, MISS_NRC, 4.8, live_did=0xF190))
        recorder.close()
        self.assertEqual(self.records("ecu")[0]["live_did"], "F190")

    def test_an_ecu_with_no_known_good_did_records_a_null_live_did(self):
        recorder = Recorder(self.path)
        recorder.ecu(EcuInfo("7E4", None, MISS_NRC, 4.8))
        recorder.close()
        self.assertIsNone(self.records("ecu")[0]["live_did"])

    def test_every_row_is_flushed_so_an_interrupted_run_is_resumable(self):
        # The whole point of --resume: a row must be on disk the moment it is
        # written, not when the file is closed.
        recorder = Recorder(self.path)
        recorder.run(firmware="x", timeout_ms=50, read_timeout_ms=500,
                     ranges=[], full=False)
        self.assertEqual(len(self.records()), 1)   # readable before close()
        recorder.close()


class StallingBoard(FakeBoard):
    """A board that stops mid-reply, the way the real one did on the car.

    Twice in one session the ESP32-S3 simply stopped writing to USB CDC part
    way through answering: once with nothing at all (`got b''`), once after
    five bytes of a perfectly ordinary negative response (`got b'NRC 3'`). The
    CAN side was healthy both times -- this is the board's serial link, not the
    car -- and the board answered ATI normally seconds later.

    `stall_on` counts DID requests only, so a test can stall a chosen probe
    without having to count the AT traffic around it.
    """

    def __init__(self, *, stall_on=(1,), partial="", stall_at_commands=False,
                 **kwargs):
        super().__init__(**kwargs)
        self._requests = 0
        self._stall_on = set(stall_on)
        self._partial = partial
        self._stall_at_commands = stall_at_commands

    def _reply(self, line: str) -> str:
        upper = line.upper()
        if upper.startswith("AT"):
            if self._stall_at_commands:
                self.sent.append(line)
                return self._partial        # never returns the prompt
            return super()._reply(line)
        self._requests += 1
        if self._requests in self._stall_on:
            self.sent.append(line)
            return self._partial
        return super()._reply(line)


class TestSweepSurvivesAStalledBoard(SweepTestCase):
    """A stalled serial link must not end a run that has hours of work in it.

    Before this, an AdapterError raised inside the sweep escaped all the way to
    main()'s `except AdapterError`, which reports it as "init failed" -- the
    wrong diagnosis, pointing at firmware and setup rather than at one dropped
    reply thousands of rows in. It also skipped recorder.end(), recorder.close()
    and the "re-run with --resume" hint. On the car this happened roughly every
    6,000 probes, so a 46,000-pair scan could not run unattended.
    """

    def test_a_stall_does_not_end_the_run(self):
        board = StallingBoard(
            stall_on=(1,), table={("7E4", 0x0101): bytes.fromhex("620101AA")})
        counts = self.run_sweep(board, [0x0101])
        row = self.records("did")[0]
        self.assertEqual(row["status"], POSITIVE)
        self.assertEqual(counts["positive"], 1)

    def test_a_truncated_reply_is_survived_too(self):
        # The `got b'NRC 3'` case: bytes arrived, the prompt never did.
        board = StallingBoard(
            stall_on=(1,), partial="NRC 3",
            table={("7E4", 0x0101): bytes.fromhex("620101AA")})
        self.run_sweep(board, [0x0101])
        self.assertEqual(self.records("did")[0]["status"], POSITIVE)

    def test_the_board_is_re_initialised_after_a_stall(self):
        # A stalled board is in an unknown state; ATZ is how this tool puts a
        # board into a known one.
        board = StallingBoard(stall_on=(1,))
        self.run_sweep(board, [0x0101])
        self.assertIn("ATZ", board.sent)

    def test_the_recovery_re_sends_the_header_before_the_next_request(self):
        # SAFETY. ATZ resets the board's header to 0x7DF (at_parser.h), the
        # functional address the firmware accepts a reply from ANY responder
        # on -- two of them interleave into a corrupt reassembly. A recovery
        # that left the cached ATSH in place would skip re-sending it and
        # transmit the rest of this ECU's sweep on 7DF.
        board = StallingBoard(stall_on=(1,))
        self.run_sweep(board, [0x0101])
        after_reset = board.sent[board.sent.index("ATZ"):]
        header = next(c for c in after_reset if c.startswith("ATSH"))
        request = next(c for c in after_reset if c.startswith("22"))
        self.assertEqual(header, "ATSH7E4")
        self.assertLess(after_reset.index(header), after_reset.index(request))

    def test_the_recovery_restores_the_sweep_deadline(self):
        # ATZ also resets ATST to the firmware default. Left there, every
        # subsequent miss would cost the default window instead of the sweep
        # deadline the ETA was calculated from.
        board = StallingBoard(stall_on=(1,))
        self.run_sweep(board, [0x0101])
        after_reset = board.sent[board.sent.index("ATZ"):]
        self.assertIn("ATST0C", after_reset)        # 50 ms / 4 = 0x0C

    def test_a_stall_still_counts_toward_the_abort(self):
        # Recovery must not make a genuinely broken link invisible: a board
        # that stalls on every probe has to stop the run, not spin forever.
        dids = list(range(0x0100, 0x0100 + MAX_CONSECUTIVE_ERRORS))
        board = StallingBoard(stall_on=range(1, 100))
        with self.assertRaises(ScanAborted):
            self.run_sweep(board, dids)

    def test_a_board_that_never_comes_back_ends_the_run(self):
        # If the re-initialisation itself cannot reach the board, this is no
        # longer a transient stall and there is nothing to recover to.
        board = StallingBoard(stall_on=(1,), stall_at_commands=True)
        with self.assertRaises(AdapterError):
            self.run_sweep(board, [0x0101])


class TestAnUnrecoverableStallIsReportedAsAStop(SweepTestCase):
    """What main() does when the sweep cannot be rescued.

    A stall that survives re-initialisation ends the run -- but it must end it
    the way an abort ends it, with the log closed and the recovery route named.
    Reporting it as "init failed" sends the operator to the firmware and the
    port when the actual state is a complete, resumable log holding thousands
    of rows.
    """

    def drive(self, exc):
        def raise_it(*args, **kwargs):
            raise exc

        board = FakeBoard()
        err = io.StringIO()
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(
                did_scan, "open_serial", lambda *a, **k: board))
            stack.enter_context(mock.patch.object(
                did_scan, "discover_ecus",
                lambda *a, **k: [EcuInfo("7E4", None, MISS_NRC, 1.0)]))
            stack.enter_context(mock.patch.object(did_scan, "sweep", raise_it))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            stack.enter_context(contextlib.redirect_stderr(err))
            status = did_scan.main(["--port", "/dev/null", "--out", self.path,
                                    "--range", "0100-0101"])
        return status, err.getvalue()

    def test_a_mid_sweep_stall_is_not_reported_as_an_init_failure(self):
        _, err = self.drive(AdapterError(
            "timed out waiting for the prompt after '2201B8'; got b''"))
        self.assertNotIn("init failed", err)
        self.assertIn("2201B8", err)

    def test_it_names_resume_as_the_way_back(self):
        _, err = self.drive(AdapterError("timed out"))
        self.assertIn("--resume", err)

    def test_the_log_is_closed_with_an_end_record(self):
        # Every row is flushed as written, so the rows survive regardless --
        # but a log with no end record cannot be told from one whose process
        # was killed, and --summary readers lose the reason it stopped.
        self.drive(AdapterError("timed out"))
        end = self.records("end")
        self.assertEqual(len(end), 1)
        self.assertIn("timed out", end[0]["aborted"])

    def test_it_still_exits_non_zero(self):
        # Distinct from ScanAborted, which is a deliberate stop: an adapter
        # that cannot be re-initialised is a genuine failure of the run.
        status, _ = self.drive(AdapterError("timed out"))
        self.assertEqual(status, 1)

    def test_a_real_init_failure_is_still_reported_as_one(self):
        # The label must keep meaning what it says: initialise() failing
        # before the sweep is still an init failure.
        board = FakeBoard(banner="ELM327 v2.1")
        err = io.StringIO()
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(
                did_scan, "open_serial", lambda *a, **k: board))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            stack.enter_context(contextlib.redirect_stderr(err))
            status = did_scan.main(["--port", "/dev/null", "--out", self.path])
        self.assertEqual(status, 1)
        self.assertIn("init failed", err.getvalue())


class TestResumeReusesRecordedEcus(SweepTestCase):
    """--resume must not pay for discovery twice.

    Discovery is 231 addresses at the generous read timeout -- about four
    minutes -- and it re-derives facts the log already holds. On the car that
    made recovery from an aborted run cost more than the run recovered: three
    successive resumes spent twelve minutes rediscovering to buy a few hundred
    new rows.
    """

    def seed_log(self, *lines: str) -> None:
        with open(self.path, "w", encoding="utf-8") as handle:
            handle.write("\n".join(lines) + "\n")

    def drive_main(self, argv, *, discover=None, board=None):
        """Run main() against a FakeBoard, capturing what sweep() is handed."""
        seen: dict = {}

        def fake_sweep(adapter, ecus, dids, recorder, **kwargs):
            seen["ecus"] = list(ecus)
            seen["skip"] = kwargs["skip"]
            return {"probes": 0}

        def refuse_discovery(*args, **kwargs):
            raise AssertionError("discovery ran when the log already had ECUs")

        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(
                did_scan, "open_serial", lambda *a, **k: board or FakeBoard()))
            stack.enter_context(mock.patch.object(
                did_scan, "discover_ecus", discover or refuse_discovery))
            stack.enter_context(mock.patch.object(did_scan, "sweep", fake_sweep))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            stack.enter_context(contextlib.redirect_stderr(io.StringIO()))
            seen["status"] = did_scan.main(argv)
        return seen

    def test_rebuilds_its_ecu_list_from_the_log_instead_of_the_car(self):
        self.seed_log(
            '{"type":"ecu","ecu":"7E4","vin":null,"miss_mode":"NRC",'
            '"miss_ms":7.1,"live_did":"F190"}',
            '{"type":"did","ecu":"7E4","did":"0100","status":"NEGATIVE","nrc":"31"}',
        )
        seen = self.drive_main(["--port", "/dev/null", "--out", self.path,
                                "--resume", "--range", "0100-0101"])
        self.assertEqual(seen["status"], 0)
        self.assertEqual([info.ecu for info in seen["ecus"]], ["7E4"])
        self.assertIn(("7E4", 0x0100), seen["skip"])

    def test_the_recorded_calibration_is_reused_rather_than_re_measured(self):
        # Calibration is cheap next to discovery, but re-measuring it would
        # overwrite miss_ms -- so the recorded value surviving is what proves
        # the reused row was used as-is.
        self.seed_log(
            '{"type":"ecu","ecu":"7E4","vin":"KMHKR81FBPU000001",'
            '"miss_mode":"NRC","miss_ms":7.1,"live_did":"F190"}')
        seen = self.drive_main(["--port", "/dev/null", "--out", self.path,
                                "--resume", "--range", "0100-0101"])
        info = seen["ecus"][0]
        self.assertEqual(info.miss_mode, MISS_NRC)
        self.assertEqual(info.miss_ms, 7.1)
        self.assertEqual(info.live_did, 0xF190)
        self.assertEqual(info.vin, "KMHKR81FBPU000001")

    def test_a_log_with_no_ecu_rows_still_discovers(self):
        # --resume against a log that never got past its first rows has nothing
        # to reuse, and must fall back rather than exit with no ECUs.
        self.seed_log('{"type":"run","started":"2026-07-28T01:09:59+00:00"}')
        discovered = [EcuInfo("7E4", None, MISS_NRC, 1.0)]
        seen = self.drive_main(
            ["--port", "/dev/null", "--out", self.path, "--resume",
             "--range", "0100-0101"],
            discover=lambda *a, **k: list(discovered))
        self.assertEqual([info.ecu for info in seen["ecus"]], ["7E4"])

    def test_ecu_wins_over_the_log_but_still_reuses_its_calibration(self):
        # --ecu is an explicit narrowing of this run; the log is where the
        # calibration for those addresses already lives.
        self.seed_log(
            '{"type":"ecu","ecu":"7E4","vin":null,"miss_mode":"NRC",'
            '"miss_ms":7.1,"live_did":"F190"}',
            '{"type":"ecu","ecu":"7C6","vin":null,"miss_mode":"NRC",'
            '"miss_ms":19.5}')
        seen = self.drive_main(["--port", "/dev/null", "--out", self.path,
                                "--resume", "--ecu", "7C6",
                                "--range", "0100-0101"])
        self.assertEqual([info.ecu for info in seen["ecus"]], ["7C6"])
        self.assertEqual(seen["ecus"][0].miss_ms, 19.5)

    def test_without_resume_the_log_is_ignored_and_discovery_runs(self):
        # A fresh run must never inherit an old log's ECU list: the car may not
        # be the same car, and nothing has been asked to be continued.
        self.seed_log(
            '{"type":"ecu","ecu":"7E4","vin":null,"miss_mode":"NRC","miss_ms":7.1}')
        discovered = [EcuInfo("7C6", None, MISS_NRC, 2.0)]
        seen = self.drive_main(
            ["--port", "/dev/null", "--out", self.path, "--range", "0100-0101"],
            discover=lambda *a, **k: list(discovered))
        self.assertEqual([info.ecu for info in seen["ecus"]], ["7C6"])


class TestSummarise(SweepTestCase):
    """summarise() reads a JSONL log and renders CSV; no board involved."""

    def write_log(self, lines: list[str]) -> None:
        with open(self.path, "w", encoding="utf-8") as handle:
            handle.write("\n".join(lines) + "\n")

    def summary_of(self, lines: list[str]) -> str:
        self.write_log(lines)
        out = io.StringIO()
        status = summarise(self.path, out)
        self.assertEqual(status, 0)
        return out.getvalue()

    def test_a_confirmed_positive_row_has_an_empty_note(self):
        csv = self.summary_of([
            json.dumps({"type": "did", "ecu": "7E4", "did": "0101",
                        "status": "POSITIVE", "len": 10}),
        ])
        self.assertEqual(csv, "header,pid,payload_bytes,note\n"
                               "7E4,220101,10,\n")

    def test_an_unstable_row_gets_the_unstable_note(self):
        csv = self.summary_of([
            json.dumps({"type": "did", "ecu": "7E4", "did": "0105",
                        "status": "POSITIVE", "len": 56, "unstable": True}),
        ])
        self.assertEqual(csv, "header,pid,payload_bytes,note\n"
                               "7E4,220105,56,unstable\n")

    def test_an_unconfirmed_row_gets_the_unconfirmed_note(self):
        # Distinct from "unstable": the Phase 4b re-read never came back
        # POSITIVE at all, so this payload was never confirmed at the
        # generous timeout. Collapsing it into an empty note (the pre-fix
        # bug) would make an unconfirmed hit look identical to a fully
        # confirmed one.
        csv = self.summary_of([
            json.dumps({"type": "did", "ecu": "7E4", "did": "0110",
                        "status": "POSITIVE", "len": 4, "unconfirmed": True}),
        ])
        self.assertEqual(csv, "header,pid,payload_bytes,note\n"
                               "7E4,220110,4,unconfirmed\n")

    def test_session_gated_nrcs_appear_only_in_the_gated_block(self):
        csv = self.summary_of([
            json.dumps({"type": "did", "ecu": "7A0", "did": "C00B",
                        "status": "NEGATIVE", "nrc": "7E"}),
            json.dumps({"type": "did", "ecu": "7A0", "did": "C00C",
                        "status": "NEGATIVE", "nrc": "7F"}),
        ])
        lines = csv.splitlines()
        self.assertEqual(lines[0], "header,pid,payload_bytes,note")
        self.assertEqual(
            lines[1],
            "# 2 session-gated DIDs (exist but need an extended session, "
            "which this tool does not enter):")
        self.assertEqual(lines[2], "# 7A0,22C00B")
        self.assertEqual(lines[3], "# 7A0,22C00C")
        self.assertEqual(len(lines), 4)   # no CSV data row for either

    def test_non_did_record_types_are_ignored(self):
        csv = self.summary_of([
            json.dumps({"type": "run", "started": "x", "ecu": "7E4",
                        "did": "0101", "status": "POSITIVE", "len": 1}),
            json.dumps({"type": "ecu", "ecu": "7E4", "did": "0101",
                        "status": "POSITIVE"}),
            json.dumps({"type": "end", "ecu": "7E4", "did": "0101",
                        "status": "POSITIVE", "probes": 1}),
        ])
        self.assertEqual(csv, "header,pid,payload_bytes,note\n")

    def test_a_malformed_json_line_is_skipped_without_crashing(self):
        csv = self.summary_of([
            "{not valid json",
            json.dumps({"type": "did", "ecu": "7E4", "did": "0101",
                        "status": "POSITIVE", "len": 1}),
        ])
        self.assertEqual(csv, "header,pid,payload_bytes,note\n"
                               "7E4,220101,1,\n")

    def test_rows_are_sorted_by_ecu_then_did(self):
        csv = self.summary_of([
            json.dumps({"type": "did", "ecu": "7E4", "did": "0200",
                        "status": "POSITIVE", "len": 1}),
            json.dumps({"type": "did", "ecu": "7A0", "did": "0100",
                        "status": "POSITIVE", "len": 1}),
            json.dumps({"type": "did", "ecu": "7E4", "did": "0100",
                        "status": "POSITIVE", "len": 1}),
        ])
        self.assertEqual(csv.splitlines()[1:], [
            "7A0,220100,1,",
            "7E4,220100,1,",
            "7E4,220200,1,",
        ])

    def test_a_log_with_no_did_rows_produces_header_only(self):
        csv = self.summary_of([
            json.dumps({"type": "run", "started": "x"}),
            json.dumps({"type": "end", "probes": 0}),
        ])
        self.assertEqual(csv, "header,pid,payload_bytes,note\n")

    def test_a_did_row_missing_ecu_or_did_is_skipped_not_crashed(self):
        csv = self.summary_of([
            json.dumps({"type": "did", "did": "0101",
                        "status": "POSITIVE", "len": 1}),
            json.dumps({"type": "did", "ecu": "7E4",
                        "status": "POSITIVE", "len": 1}),
            json.dumps({"type": "did", "ecu": "7E4", "did": "0101",
                        "status": "POSITIVE", "len": 1}),
        ])
        self.assertEqual(csv, "header,pid,payload_bytes,note\n"
                               "7E4,220101,1,\n")

    def test_a_json_line_that_is_not_an_object_is_skipped_not_crashed(self):
        # json.loads succeeding does not mean a dict came back. Every one of
        # these is a valid JSON document whose .get() raises AttributeError, and
        # a truncated write can leave one behind -- --summary is what an
        # operator reaches for when a run went wrong, so it has to survive them.
        csv = self.summary_of([
            "42", '"hello"', "[1,2,3]", "null", "true",
            json.dumps({"type": "did", "ecu": "7E4", "did": "0101",
                        "status": "POSITIVE", "len": 1}),
        ])
        self.assertEqual(csv, "header,pid,payload_bytes,note\n"
                               "7E4,220101,1,\n")

    def test_a_nonexistent_log_prints_a_clean_error_and_returns_nonzero(self):
        missing = self.path + ".does-not-exist"
        out = io.StringIO()
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            status = summarise(missing, out)
        self.assertEqual(status, 1)
        self.assertEqual(out.getvalue(), "")
        self.assertIn(missing, err.getvalue())


from did_scan import (
    MAX_TIMEOUT_MS,
    MIN_TIMEOUT_MS,
    SERIAL_READ_MARGIN_S,
    serial_timeout_s,
)


class TestGenerousTimeout(unittest.TestCase):
    """The context manager may only ever widen the deadline.

    Nothing orders --timeout against --read-timeout, and both
    `--timeout 1000 --read-timeout 100` are inside the bounds the tool enforces.
    Setting ATST from read_timeout_ms unconditionally would then make discovery
    and the liveness probe run BELOW the sweep value -- reintroducing the
    dropped-ECU finding on precisely the two paths whose failure costs a whole
    ECU rather than a single DID.
    """

    def effective_atst(self, *, timeout_ms: int, read_timeout_ms: int) -> str:
        board = FakeBoard()
        adapter = Adapter(board)
        with did_scan.generous_timeout(adapter, timeout_ms=timeout_ms,
                                       read_timeout_ms=read_timeout_ms):
            inside = list(board.sent)
        return inside[-1]

    def test_an_inverted_flag_pair_still_widens_the_deadline(self):
        # max(1000, 100) = 1000 ms -> 250 units -> ATSTFA. Setting it from
        # read_timeout_ms alone would give 100 ms -> ATST19, i.e. a deadline
        # NARROWER than the sweep's, which is the bug.
        self.assertEqual(self.effective_atst(timeout_ms=1000,
                                             read_timeout_ms=100),
                         "ATSTFA")

    def test_the_normal_ordering_is_unchanged(self):
        self.assertEqual(self.effective_atst(timeout_ms=50,
                                             read_timeout_ms=500),
                         "ATST7D")

    def test_equal_values_are_a_no_op_in_effect(self):
        self.assertEqual(self.effective_atst(timeout_ms=200,
                                             read_timeout_ms=200),
                         "ATST32")

    def test_the_sweep_value_is_what_gets_restored(self):
        # The restore is always the sweep deadline, inverted pair or not: that
        # is what the sweep's per-miss cost and its ETA were calculated from.
        board = FakeBoard()
        with did_scan.generous_timeout(Adapter(board), timeout_ms=1000,
                                       read_timeout_ms=100):
            pass
        self.assertEqual(board.sent, ["ATSTFA", "ATSTFA"])

    def test_discovery_with_an_inverted_pair_still_finds_a_slow_vin(self):
        # The property that actually matters, end to end: a VIN needing 600 ms
        # is found when --timeout is the larger flag, because discovery runs at
        # the wider of the two.
        board = DeadlineBoard(needs_ms=600, table={
            ("7E4", 0xF190): b"\x62\xF1\x90" + b"KMHKR81FBPU000001",
        })
        found = discover(Adapter(board), ["7E4"],
                         timeout_ms=1000, read_timeout_ms=100)
        self.assertEqual([i.ecu for i in found], ["7E4"])
        self.assertEqual(found[0].vin, "KMHKR81FBPU000001")


class TestSerialTimeout(unittest.TestCase):
    def test_it_covers_whichever_board_deadline_is_larger(self):
        # pyserial must never be the thing that gives up first. It used to be
        # hardcoded at 5 s, so --read-timeout 6000 had the board wait 6000 ms
        # while pyserial abandoned the read at 5000 -- an AdapterError out of
        # sweep(), caught by main()'s outer handler and printed as "init
        # failed", with recorder.end() and recorder.close() never running.
        self.assertEqual(serial_timeout_s(50, 6000),
                         6.0 + SERIAL_READ_MARGIN_S)
        self.assertEqual(serial_timeout_s(9000, 500),
                         9.0 + SERIAL_READ_MARGIN_S)

    def test_it_leaves_headroom_over_the_board_deadline(self):
        self.assertGreater(serial_timeout_s(500, 500), 0.5)


class TestTimeoutBounds(unittest.TestCase):
    """Both flags are bounded at both ends, for one reason: the timeout the tool
    prints and records has to be the timeout the board is actually using."""

    def refuse(self, argv: list[str]) -> str:
        err = io.StringIO()
        with contextlib.redirect_stderr(err), \
                contextlib.redirect_stdout(io.StringIO()):
            status = did_scan.main(argv)
        self.assertEqual(status, 2)
        return err.getvalue()

    def test_a_timeout_above_the_firmware_ceiling_is_refused(self):
        # at_parser.cpp clamps ATST to kMaxTimeoutMs (65535). Above that the
        # board silently uses a different deadline than the one reported.
        message = self.refuse(["--port", "/dev/null",
                               "--timeout", str(MAX_TIMEOUT_MS + 1)])
        self.assertIn("--timeout", message)
        self.assertIn(str(MAX_TIMEOUT_MS), message)

    def test_a_read_timeout_above_the_firmware_ceiling_is_refused(self):
        message = self.refuse(["--port", "/dev/null",
                               "--read-timeout", str(MAX_TIMEOUT_MS + 1)])
        self.assertIn("--read-timeout", message)

    def test_the_existing_floor_still_holds(self):
        message = self.refuse(["--port", "/dev/null",
                               "--timeout", str(MIN_TIMEOUT_MS - 1)])
        self.assertIn(str(MIN_TIMEOUT_MS), message)

    def test_the_bounds_are_checked_before_any_port_is_opened(self):
        # A bad flag must not touch the serial device at all.
        with mock.patch.object(did_scan, "open_serial") as opener:
            self.refuse(["--port", "/dev/null",
                         "--read-timeout", str(MAX_TIMEOUT_MS + 1)])
        opener.assert_not_called()


if __name__ == "__main__":
    unittest.main()
