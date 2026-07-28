"""Host tests for didscan_core. Run: python3 -m unittest test_didscan_core -v"""

import unittest

from didscan_core import ADAPTER_ERROR, NEGATIVE, NO_RESPONSE, POSITIVE, classify
from didscan_core import DEFAULT_RANGES, RangeError, expand_ranges
from didscan_core import already_scanned, recorded_ecus


class TestExpandRanges(unittest.TestCase):
    def test_single_did(self):
        self.assertEqual(expand_ranges(["F190"]), [0xF190])

    def test_inclusive_range(self):
        self.assertEqual(expand_ranges(["0100-0103"]), [0x0100, 0x0101, 0x0102, 0x0103])

    def test_multiple_specs_concatenate_in_order(self):
        self.assertEqual(expand_ranges(["0101", "B000-B001"]), [0x0101, 0xB000, 0xB001])

    def test_overlapping_specs_are_deduplicated(self):
        # Two ranges that overlap must not probe the same DID twice; a duplicate
        # would be recorded as two findings for one identifier.
        self.assertEqual(expand_ranges(["0100-0102", "0101-0103"]),
                         [0x0100, 0x0101, 0x0102, 0x0103])

    def test_case_insensitive(self):
        self.assertEqual(expand_ranges(["f190"]), [0xF190])

    def test_full_space_is_expressible(self):
        self.assertEqual(len(expand_ranges(["0000-FFFF"])), 65536)

    def test_default_ranges_are_valid_and_cover_the_known_pids(self):
        dids = set(expand_ranges(DEFAULT_RANGES))
        # Every DID the app polls today must be inside the default sweep, or a
        # default run would fail to confirm what we already rely on.
        for known in (0x0101, 0x0105, 0xE011, 0x0100, 0xC00B, 0xB002):
            self.assertIn(known, dids)

    def test_reversed_range_is_an_error(self):
        with self.assertRaises(RangeError):
            expand_ranges(["0200-0100"])

    def test_non_hex_is_an_error(self):
        with self.assertRaises(RangeError):
            expand_ranges(["ZZZZ"])

    def test_out_of_range_is_an_error(self):
        with self.assertRaises(RangeError):
            expand_ranges(["10000"])

    def test_empty_spec_is_an_error(self):
        with self.assertRaises(RangeError):
            expand_ranges([""])


class TestClassify(unittest.TestCase):
    def test_single_frame_positive(self):
        # ATS0 is set, so no spaces. 62 01 05 + 4 payload bytes = 7 bytes, which
        # formatter.cpp emits as one line with NO length header.
        r = classify("62010500000000")
        self.assertEqual(r.status, POSITIVE)
        self.assertEqual(r.payload, bytes.fromhex("62010500000000"))

    def test_multi_frame_positive_is_reassembled(self):
        # Fixture from test_formatter's test_multi_frame_without_spaces.
        r = classify("008\r0:62010500000000\r1:00")
        self.assertEqual(r.status, POSITIVE)
        self.assertEqual(r.payload, bytes.fromhex("6201050000000000"))

    def test_multi_frame_with_spaces_is_reassembled(self):
        # Fixture from test_formatter's test_multi_frame_uses_length_header...
        raw = ("014\r"
               "0: 62 01 01 FF F7 E7 FF\r"
               "1: 8C 00 4B 00 00 00 00\r"
               "2: 00 00 00 00 00 00")
        r = classify(raw)
        self.assertEqual(r.status, POSITIVE)
        self.assertEqual(len(r.payload), 0x14)
        self.assertTrue(r.payload.startswith(bytes.fromhex("620101FFF7E7FF")))

    def test_declared_length_shorter_than_lines_truncates(self):
        # The %03X header is authoritative: the last CAN frame is padded to 8
        # bytes, so trailing bytes beyond the declared length are padding, not
        # data. Recording them would inflate every multi-frame payload.
        r = classify("003\r0:620101AABBCC")
        self.assertEqual(r.payload, bytes.fromhex("620101"))

    def test_declared_length_longer_than_lines_is_an_error(self):
        # A short read means the reply was cut off; treating it as a payload
        # would record a truncated value as complete.
        r = classify("020\r0:620101")
        self.assertEqual(r.status, ADAPTER_ERROR)

    def test_short_middle_line_is_an_error(self):
        # Middle line 1 carries only 6 bytes instead of 7, but the total
        # still reaches the declared length of 0x14 (7 + 6 + 7). Accepting
        # this would shift every byte after the short line out of position
        # while still looking complete — a wrong payload marked POSITIVE.
        raw = ("014\r"
               "0:620101FFF7E7FF\r"
               "1:8C004B000000\r"
               "2:00000000000000")
        r = classify(raw)
        self.assertEqual(r.status, ADAPTER_ERROR)

    def test_final_line_shorter_than_seven_bytes_is_positive(self):
        # The last line is allowed to be short - that is the normal case,
        # since the final CAN frame rarely divides evenly by 7.
        r = classify("008\r0:62010500000000\r1:00")
        self.assertEqual(r.status, POSITIVE)
        self.assertEqual(r.payload, bytes.fromhex("6201050000000000"))

    def test_lone_length_line_is_an_error(self):
        # A single 4-hex-digit line matches the length-header pattern, but
        # formatter.cpp only ever emits a length header ahead of indexed data
        # lines. Treating it as a bare 2-byte payload would be a silent
        # misparse of what is really a truncated multi-frame reply.
        r = classify("0105")
        self.assertEqual(r.status, ADAPTER_ERROR)

    def test_index_prefix_wrapping_past_f_is_handled(self):
        # formatter.cpp masks the index with 0xF, so line 16 is "0:" again.
        # Ordering must come from line order, never from the prefix.
        lines = ["077"] + [f"{i & 0xF:X}:" + "AA" * 7 for i in range(17)]
        r = classify("\r".join(lines))
        self.assertEqual(r.status, POSITIVE)
        self.assertEqual(r.payload, b"\xAA" * 0x77)  # 17 lines x 7 bytes

    def test_nrc_request_out_of_range(self):
        r = classify("NRC 31")
        self.assertEqual(r.status, NEGATIVE)
        self.assertEqual(r.nrc, 0x31)

    def test_nrc_session_gated(self):
        self.assertEqual(classify("NRC 7E").nrc, 0x7E)
        self.assertEqual(classify("NRC 7F").nrc, 0x7F)

    def test_no_data_is_no_response(self):
        self.assertEqual(classify("NO DATA").status, NO_RESPONSE)

    def test_adapter_faults(self):
        for fault in ("CAN ERROR", "?", "BUFFER FULL", "UNABLE TO CONNECT"):
            self.assertEqual(classify(fault).status, ADAPTER_ERROR, fault)

    def test_prompt_and_whitespace_are_stripped(self):
        # The caller reads up to the prompt; be tolerant if it is left on.
        self.assertEqual(classify("NO DATA\r>").status, NO_RESPONSE)
        self.assertEqual(classify("  62010500000000  ").status, POSITIVE)

    def test_odd_hex_digit_count_is_an_error(self):
        self.assertEqual(classify("6201050").status, ADAPTER_ERROR)

    def test_unrecognised_text_is_an_error(self):
        self.assertEqual(classify("SEARCHING...").status, ADAPTER_ERROR)

    def test_empty_reply_is_an_error(self):
        self.assertEqual(classify("").status, ADAPTER_ERROR)

    def test_raw_is_always_preserved(self):
        self.assertEqual(classify("NRC 31").raw, "NRC 31")


class TestAlreadyScanned(unittest.TestCase):
    def test_collects_ecu_did_pairs(self):
        lines = [
            '{"type":"run","started":"x"}',
            '{"type":"did","ecu":"7E4","did":"0101","status":"POSITIVE"}',
            '{"type":"did","ecu":"7E4","did":"0102","status":"NEGATIVE"}',
        ]
        self.assertEqual(already_scanned(lines), {("7E4", 0x0101), ("7E4", 0x0102)})

    def test_ignores_non_did_records(self):
        lines = ['{"type":"ecu","ecu":"7E4","miss_mode":"NRC_31"}']
        self.assertEqual(already_scanned(lines), set())

    def test_tolerates_a_truncated_final_line(self):
        # A run killed mid-write leaves a partial line. Resume must not crash on
        # the very file it exists to recover from.
        lines = [
            '{"type":"did","ecu":"7E4","did":"0101","status":"POSITIVE"}',
            '{"type":"did","ecu":"7E4","did":"010',
        ]
        self.assertEqual(already_scanned(lines), {("7E4", 0x0101)})

    def test_tolerates_blank_lines(self):
        self.assertEqual(already_scanned(["", "  "]), set())

    def test_case_insensitive_did_and_ecu(self):
        lines = ['{"type":"did","ecu":"7e4","did":"f190","status":"POSITIVE"}']
        self.assertEqual(already_scanned(lines), {("7E4", 0xF190)})

    def test_a_json_line_that_is_not_an_object_is_skipped(self):
        # The docstring promises this function "silently skips anything
        # unparseable", and json.loads succeeding is not the same as an object
        # coming back: every line below is a valid JSON document whose .get()
        # raises AttributeError. A truncated write can leave one, and the file
        # this reads is by definition one an interrupted run was mid-write on.
        lines = [
            "42",
            '"hello"',
            "[1,2,3]",
            "null",
            "true",
            '{"type":"did","ecu":"7E4","did":"0101","status":"POSITIVE"}',
        ]
        self.assertEqual(already_scanned(lines), {("7E4", 0x0101)})

    def test_an_adapter_error_row_is_not_in_the_skip_set(self):
        # ADAPTER_ERROR carries no finding -- it is what classify() returns for
        # a garbled or truncated reply, including a real multi-frame hit that
        # lost a byte. Burying it forever on --resume would let a genuine DID
        # vanish from the inventory with no trace, so a resumed run must
        # re-probe it rather than skip it.
        lines = [
            '{"type":"did","ecu":"7E4","did":"0101","status":"ADAPTER_ERROR"}',
        ]
        self.assertEqual(already_scanned(lines), set())

    def test_only_the_positive_pair_is_skipped_alongside_an_adapter_error(self):
        lines = [
            '{"type":"did","ecu":"7E4","did":"0101","status":"POSITIVE"}',
            '{"type":"did","ecu":"7E4","did":"0102","status":"ADAPTER_ERROR"}',
        ]
        self.assertEqual(already_scanned(lines), {("7E4", 0x0101)})


class TestRecordedEcus(unittest.TestCase):
    """--resume rebuilding its ECU list from the log instead of the car.

    Discovery costs a four-minute sweep of 231 addresses, and everything it
    learns is already written to the log as type:"ecu" rows. Re-running it on
    every --resume is what made an aborted scan expensive to recover from.
    """

    def test_reads_the_calibration_of_each_recorded_ecu(self):
        lines = [
            '{"type":"run","started":"2026-07-28T01:09:59+00:00"}',
            '{"type":"ecu","ecu":"7E4","vin":null,"miss_mode":"NRC",'
            '"miss_ms":7.1,"live_did":"F100"}',
            '{"type":"ecu","ecu":"7C6","vin":"7YAKMDDC6SY015480",'
            '"miss_mode":"TIMEOUT","miss_ms":19.5,"live_did":"F190"}',
        ]
        self.assertEqual(recorded_ecus(lines), [
            {"ecu": "7E4", "vin": None, "miss_mode": "NRC",
             "miss_ms": 7.1, "live_did": 0xF100},
            {"ecu": "7C6", "vin": "7YAKMDDC6SY015480", "miss_mode": "TIMEOUT",
             "miss_ms": 19.5, "live_did": 0xF190},
        ])

    def test_a_later_calibration_replaces_an_earlier_one_in_place(self):
        # Every run appends its own ecu rows, so a log that has been resumed
        # twice holds three copies of each ECU. The freshest calibration is the
        # one to keep -- and the ECU must not appear three times in the sweep.
        lines = [
            '{"type":"ecu","ecu":"7E4","vin":null,"miss_mode":"NRC","miss_ms":7.1}',
            '{"type":"ecu","ecu":"7C6","vin":null,"miss_mode":"NRC","miss_ms":19.5}',
            '{"type":"ecu","ecu":"7E4","vin":null,"miss_mode":"NRC","miss_ms":34.5}',
        ]
        self.assertEqual([(r["ecu"], r["miss_ms"]) for r in recorded_ecus(lines)],
                         [("7E4", 34.5), ("7C6", 19.5)])

    def test_an_old_log_without_live_did_still_yields_its_ecus(self):
        # live_did is written only by versions that record it. A log from
        # before that must still resume, just without the liveness shortcut.
        lines = [
            '{"type":"ecu","ecu":"7E4","vin":null,"miss_mode":"NRC","miss_ms":7.1}',
        ]
        self.assertEqual(recorded_ecus(lines)[0]["live_did"], None)

    def test_non_ecu_rows_and_unusable_lines_are_ignored(self):
        lines = [
            "",
            "{not json",
            "42",
            '{"type":"did","ecu":"7E4","did":"0101","status":"POSITIVE"}',
            '{"type":"ecu","miss_mode":"NRC","miss_ms":1.0}',
            '{"type":"ecu","ecu":"7E4","vin":null,"miss_mode":"NRC","miss_ms":7.1}',
        ]
        self.assertEqual([r["ecu"] for r in recorded_ecus(lines)], ["7E4"])


if __name__ == "__main__":
    unittest.main()
