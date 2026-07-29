#include <unity.h>
#include <cstring>
#include "elm327/at_parser.h"

void setUp() {}
void tearDown() {}

void test_adapter_state_defaults_match_a_fresh_elm327() {
    AdapterState state;
    TEST_ASSERT_TRUE(state.echo);
    TEST_ASSERT_TRUE(state.spaces);
    TEST_ASSERT_FALSE(state.headers);
    TEST_ASSERT_TRUE(state.autoFormat);
    TEST_ASSERT_EQUAL_UINT16(0x7DF, state.header);
}

void test_recognises_at_prefix() {
    TEST_ASSERT_TRUE(isAtCommand("ATZ"));
    TEST_ASSERT_TRUE(isAtCommand("atz"));
    TEST_ASSERT_TRUE(isAtCommand("AT SH 7E4"));
    TEST_ASSERT_FALSE(isAtCommand("0100"));
    TEST_ASSERT_FALSE(isAtCommand("220101"));
}

void test_recognises_the_at_sign_command_prefix() {
    // @1/@2/@3 are ELM327 commands with no AT prefix. Without this, "@1"
    // falls through to runRequest(), fails hex parsing, and answers '?' —
    // which looks to a client exactly like an unsupported command.
    TEST_ASSERT_TRUE(isAtCommand("@1"));
    TEST_ASSERT_TRUE(isAtCommand("@2"));
    TEST_ASSERT_TRUE(isAtCommand(" @3 ABCDEF012345"));
    // A hex request line still must not be mistaken for a command.
    TEST_ASSERT_FALSE(isAtCommand("0100"));
    TEST_ASSERT_FALSE(isAtCommand("220101"));
}

void test_toggles_echo_spaces_headers_linefeeds() {
    AdapterState s;
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATE0", s));
    TEST_ASSERT_FALSE(s.echo);
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATS0", s));
    TEST_ASSERT_FALSE(s.spaces);
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATH1", s));
    TEST_ASSERT_TRUE(s.headers);
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATL0", s));
    TEST_ASSERT_FALSE(s.linefeeds);
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATCAF0", s));
    TEST_ASSERT_FALSE(s.autoFormat);
}

void test_sets_header_from_atsh() {
    AdapterState s;
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATSH 7E4", s));
    TEST_ASSERT_EQUAL_UINT16(0x7E4, s.header);
    // ELM327 accepts the form without a space too.
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATSH7A0", s));
    TEST_ASSERT_EQUAL_UINT16(0x7A0, s.header);
}

void test_sets_timeout_from_atst_in_four_ms_units() {
    AdapterState s;
    // ATST32 = 0x32 * 4 ms = 200 ms. This is the value the Android app sends.
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATST32", s));
    TEST_ASSERT_EQUAL_UINT16(200, s.timeoutMs);
}

void test_atst_zero_is_clamped_to_a_usable_floor() {
    AdapterState s;
    // ATST00 asks for a 0 ms timeout. Honouring it would make EVERY request
    // return NO DATA before the car could answer — the adapter would look
    // permanently dead. A real ELM327 substitutes its default instead.
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATST00", s));
    TEST_ASSERT_EQUAL_UINT16(kMinTimeoutMs, s.timeoutMs);

    // Anything below the floor is raised to it: 0x02 * 4 ms = 8 ms.
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATST02", s));
    TEST_ASSERT_EQUAL_UINT16(kMinTimeoutMs, s.timeoutMs);

    // A value at or above the floor is left alone.
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATST0A", s));
    TEST_ASSERT_EQUAL_UINT16(40, s.timeoutMs);
}

void test_atst_large_value_cannot_wrap_back_under_the_floor() {
    AdapterState s;
    // 0x4000 * 4 = 65536, which truncates to 0 in a uint16 — straight back to
    // the instant-NO-DATA failure the floor exists to prevent.
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATST4000", s));
    TEST_ASSERT_TRUE(s.timeoutMs >= kMinTimeoutMs);
    TEST_ASSERT_EQUAL_UINT16(kMaxTimeoutMs, s.timeoutMs);
}

void test_rejects_a_header_above_eleven_bits() {
    AdapterState s;
    const uint16_t before = s.header;
    // 11-bit CAN IDs only. 0x800 does not fit and must not be truncated into
    // some other ECU's address.
    TEST_ASSERT_EQUAL(AtResult::Unknown, applyAtCommand("ATSH800", s));
    TEST_ASSERT_EQUAL(AtResult::Unknown, applyAtCommand("ATSH1FFF", s));
    TEST_ASSERT_EQUAL_UINT16(before, s.header);
}

void test_rejects_a_header_with_trailing_garbage() {
    AdapterState s;
    const uint16_t before = s.header;
    // strtoul would happily parse "7E4" and stop at 'Z'. Accepting that would
    // silently address 0x7E4 for a command the client did not mean.
    TEST_ASSERT_EQUAL(AtResult::Unknown, applyAtCommand("ATSH7E4Z", s));
    TEST_ASSERT_EQUAL_UINT16(before, s.header);
    // A bare ATSH with no value is not a header either.
    TEST_ASSERT_EQUAL(AtResult::Unknown, applyAtCommand("ATSH", s));
    TEST_ASSERT_EQUAL_UINT16(before, s.header);
}

void test_ignores_whitespace_and_case() {
    AdapterState s;
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("at e0", s));
    TEST_ASSERT_FALSE(s.echo);
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("  ATS0  ", s));
    TEST_ASSERT_FALSE(s.spaces);
}

void test_reset_restores_defaults() {
    AdapterState s;
    applyAtCommand("ATE0", s);
    applyAtCommand("ATSH 7E4", s);
    TEST_ASSERT_EQUAL(AtResult::Reset, applyAtCommand("ATZ", s));
    TEST_ASSERT_TRUE(s.echo);
    TEST_ASSERT_EQUAL_UINT16(0x7DF, s.header);
}

void test_protocol_select_is_accepted() {
    AdapterState s;
    // The Android init sends ATSP6 (ISO 15765-4 CAN 11-bit 500k), which is
    // the only protocol this firmware speaks. Accept it; accept ATSP0 (auto)
    // as equivalent since we have exactly one protocol to auto-detect.
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATSP6", s));
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATSP0", s));
    // A protocol we cannot speak must be rejected, not silently accepted.
    TEST_ASSERT_EQUAL(AtResult::Unknown, applyAtCommand("ATSP3", s));
}

void test_identify_and_voltage_are_distinct_results() {
    AdapterState s;
    TEST_ASSERT_EQUAL(AtResult::Identify, applyAtCommand("ATI", s));
    TEST_ASSERT_EQUAL(AtResult::Voltage, applyAtCommand("ATRV", s));
}

void test_unknown_command_reports_unknown() {
    AdapterState s;
    TEST_ASSERT_EQUAL(AtResult::Unknown, applyAtCommand("ATXYZZY", s));
}

void test_harmless_commands_are_accepted_without_effect() {
    // Adaptive timing, flow control, and the CAN filter/mask commands: we
    // always do the right thing internally, so accept and ignore rather than
    // answering '?' and making a client think the adapter is broken.
    //
    // "Without effect" is the load-bearing half of that claim, so assert it:
    // a future edit that made one of these commands quietly move the header or
    // the timeout would change which ECU is polled with no visible symptom.
    AdapterState s;
    s.header = 0x7E4;
    s.timeoutMs = 200;
    s.spaces = false;
    s.echo = false;
    const AdapterState before = s;

    const char* harmless[] = {"ATAT1", "ATFCSM0", "ATCF7E8", "ATCM7F8",
                              "ATCRA7E8", "ATM0"};
    for (const char* command : harmless) {
        TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand(command, s));
        TEST_ASSERT_EQUAL_UINT16(before.header, s.header);
        TEST_ASSERT_EQUAL_UINT16(before.timeoutMs, s.timeoutMs);
        TEST_ASSERT_EQUAL(before.echo, s.echo);
        TEST_ASSERT_EQUAL(before.spaces, s.spaces);
        TEST_ASSERT_EQUAL(before.headers, s.headers);
        TEST_ASSERT_EQUAL(before.linefeeds, s.linefeeds);
        TEST_ASSERT_EQUAL(before.autoFormat, s.autoFormat);
    }
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_adapter_state_defaults_match_a_fresh_elm327);
    RUN_TEST(test_recognises_at_prefix);
    RUN_TEST(test_recognises_the_at_sign_command_prefix);
    RUN_TEST(test_toggles_echo_spaces_headers_linefeeds);
    RUN_TEST(test_sets_header_from_atsh);
    RUN_TEST(test_sets_timeout_from_atst_in_four_ms_units);
    RUN_TEST(test_atst_zero_is_clamped_to_a_usable_floor);
    RUN_TEST(test_atst_large_value_cannot_wrap_back_under_the_floor);
    RUN_TEST(test_rejects_a_header_above_eleven_bits);
    RUN_TEST(test_rejects_a_header_with_trailing_garbage);
    RUN_TEST(test_ignores_whitespace_and_case);
    RUN_TEST(test_reset_restores_defaults);
    RUN_TEST(test_protocol_select_is_accepted);
    RUN_TEST(test_identify_and_voltage_are_distinct_results);
    RUN_TEST(test_unknown_command_reports_unknown);
    RUN_TEST(test_harmless_commands_are_accepted_without_effect);
    return UNITY_END();
}
