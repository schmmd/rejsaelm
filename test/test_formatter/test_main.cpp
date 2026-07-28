#include <unity.h>
#include <cctype>
#include <string>
#include "elm327/formatter.h"

void setUp() {}
void tearDown() {}

// Fixtures below are lifted from the Android side's ResponseParserTest so both
// halves are checked against ONE definition of the wire format. If you change
// anything here, change it there too.

void test_short_payload_with_spaces() {
    AdapterState s;             // ATS1 default
    const uint8_t data[] = {0x41, 0x00, 0xBE, 0x3E, 0xB8, 0x11};
    TEST_ASSERT_EQUAL_STRING("41 00 BE 3E B8 11",
                             formatPayload(data, sizeof(data), s).c_str());
}

void test_short_payload_without_spaces() {
    AdapterState s;
    s.spaces = false;           // ATS0, what the Android app sends
    const uint8_t data[] = {0x41, 0x0C, 0x1A, 0xF8};
    TEST_ASSERT_EQUAL_STRING("410C1AF8",
                             formatPayload(data, sizeof(data), s).c_str());
}

void test_multi_frame_uses_length_header_and_index_prefixes() {
    AdapterState s;             // spaces on
    // 20 bytes (0x14) — exactly the Android fixture's payload.
    const uint8_t data[] = {
        0x62, 0x01, 0x01, 0xFF, 0xF7, 0xE7, 0xFF,
        0x8C, 0x00, 0x4B, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    const std::string expected =
        "014\r"
        "0: 62 01 01 FF F7 E7 FF\r"
        "1: 8C 00 4B 00 00 00 00\r"
        "2: 00 00 00 00 00 00";
    TEST_ASSERT_EQUAL_STRING(expected.c_str(),
                             formatPayload(data, sizeof(data), s).c_str());
}

void test_multi_frame_without_spaces() {
    AdapterState s;
    s.spaces = false;
    const uint8_t data[] = {
        0x62, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    const std::string expected =
        "008\r"
        "0:62010500000000\r"   // 7 bytes = 14 hex chars
        "1:00";                // the 8th byte
    TEST_ASSERT_EQUAL_STRING(expected.c_str(),
                             formatPayload(data, sizeof(data), s).c_str());
}

void test_seven_bytes_is_still_a_single_line() {
    AdapterState s;
    const uint8_t data[] = {0x41, 0x00, 0xBE, 0x3E, 0xB8, 0x11, 0x22};
    // Boundary: 7 bytes fits one CAN frame, so no length header.
    TEST_ASSERT_EQUAL_STRING("41 00 BE 3E B8 11 22",
                             formatPayload(data, sizeof(data), s).c_str());
}

void test_eight_bytes_becomes_multi_frame() {
    AdapterState s;
    const uint8_t data[] = {0x41, 0x00, 0xBE, 0x3E, 0xB8, 0x11, 0x22, 0x33};
    const std::string expected =
        "008\r"
        "0: 41 00 BE 3E B8 11 22\r"
        "1: 33";
    TEST_ASSERT_EQUAL_STRING(expected.c_str(),
                             formatPayload(data, sizeof(data), s).c_str());
}

void test_fault_output_carries_no_framing_of_its_own() {
    // ResponseParser matches fault text against an exact table ("NO DATA",
    // "CAN ERROR", ...) after splitting on CR and trimming. So a fault must
    // come out as the bare word: no length header, no "0:" index prefix, and
    // in particular no CR or prompt — handleLine appends those, and a second
    // copy here would split the fault into an extra empty line and, worse,
    // make the reply look complete to the client one prompt early.
    for (const char* fault : {"NO DATA", "CAN ERROR", "?"}) {
        const std::string out = formatFault(fault);
        TEST_ASSERT_EQUAL_STRING(fault, out.c_str());
        TEST_ASSERT_EQUAL_size_t(std::string::npos, out.find('\r'));
        TEST_ASSERT_EQUAL_size_t(std::string::npos, out.find('\n'));
        TEST_ASSERT_EQUAL_size_t(std::string::npos, out.find('>'));
        TEST_ASSERT_EQUAL_size_t(std::string::npos, out.find(':'));
    }
}

void test_prompt_is_a_bare_greater_than() {
    TEST_ASSERT_EQUAL_STRING(">", kPrompt);
}

void test_nrc_is_a_tagged_two_digit_hex_byte() {
    // "NRC" distinguishes it from every existing fault word, and from a hex
    // payload line, so the host classifier needs no lookahead.
    TEST_ASSERT_EQUAL_STRING("NRC 31", formatNrc(0x31).c_str());
    TEST_ASSERT_EQUAL_STRING("NRC 7E", formatNrc(0x7E).c_str());
    TEST_ASSERT_EQUAL_STRING("NRC 00", formatNrc(0x00).c_str());
    TEST_ASSERT_EQUAL_STRING("NRC FF", formatNrc(0xFF).c_str());
}

void test_nrc_carries_no_framing_of_its_own() {
    // Same contract as formatFault: handleLine appends CR and the prompt, and
    // a second copy here would make the reply look complete one prompt early.
    const std::string out = formatNrc(0x31);
    TEST_ASSERT_EQUAL_size_t(std::string::npos, out.find('\r'));
    TEST_ASSERT_EQUAL_size_t(std::string::npos, out.find('\n'));
    TEST_ASSERT_EQUAL_size_t(std::string::npos, out.find('>'));
}

void test_no_nrc_can_be_mistaken_for_no_data_or_for_a_payload() {
    // session.cpp picks between formatNrc(nrc) under -DREPORT_NRC and
    // formatFault("NO DATA") otherwise. That file is excluded from env:native
    // (it pulls in Arduino.h and the TWAI driver via can_bus.h), so neither
    // branch has a host test and swapping the two would pass every gate in the
    // project. What the Android app actually depends on is pinned here instead,
    // without host-compiling session.cpp: for EVERY byte value, formatNrc's
    // output is neither a fault word the app maps to an AdapterError nor
    // anything ResponseParser.kt would accept as hex payload bytes.
    //
    // Verified against android/.../ResponseParser.kt: it uppercases, strips
    // spaces, then rejects the accumulated text unless it is even-length AND
    // all hex digits. "NRC 31" becomes "NRC31" — five characters, and N/R/C are
    // not hex digits — so it lands in Fault(GARBAGE) and never in Payload. Both
    // checks below have to hold; either one alone could be satisfied by a
    // format that the other rejects.
    const char* appFaults[] = {"NO DATA", "CAN ERROR", "STOPPED", "BUFFER FULL",
                               "UNABLE TO CONNECT", "BUS INIT", "?"};
    for (int value = 0; value <= 0xFF; ++value) {
        const std::string out = formatNrc(static_cast<uint8_t>(value));

        for (const char* fault : appFaults) {
            TEST_ASSERT_TRUE(out != formatFault(fault));
        }

        std::string cleaned;
        for (char c : out) {
            if (c != ' ') cleaned += c;
        }
        bool allHex = true;
        for (char c : cleaned) {
            if (!std::isxdigit(static_cast<unsigned char>(c))) allHex = false;
        }
        const bool evenLength = (cleaned.size() % 2) == 0;
        // Payload requires both. Failing either is what makes it GARBAGE.
        TEST_ASSERT_FALSE(evenLength && allHex);
    }
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_short_payload_with_spaces);
    RUN_TEST(test_short_payload_without_spaces);
    RUN_TEST(test_multi_frame_uses_length_header_and_index_prefixes);
    RUN_TEST(test_multi_frame_without_spaces);
    RUN_TEST(test_seven_bytes_is_still_a_single_line);
    RUN_TEST(test_eight_bytes_becomes_multi_frame);
    RUN_TEST(test_fault_output_carries_no_framing_of_its_own);
    RUN_TEST(test_prompt_is_a_bare_greater_than);
    RUN_TEST(test_nrc_is_a_tagged_two_digit_hex_byte);
    RUN_TEST(test_nrc_carries_no_framing_of_its_own);
    RUN_TEST(test_no_nrc_can_be_mistaken_for_no_data_or_for_a_payload);
    return UNITY_END();
}
