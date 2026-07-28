#include <unity.h>
#include <cstring>
#include "elm327/hexline.h"

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------- hexLineToBytes

void test_parses_a_plain_hex_line() {
    uint8_t out[8] = {0};
    size_t len = 0;
    TEST_ASSERT_TRUE(hexLineToBytes("220101", out, sizeof(out), len));
    TEST_ASSERT_EQUAL_size_t(3, len);
    const uint8_t expected[] = {0x22, 0x01, 0x01};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, out, 3);
}

void test_tolerates_embedded_spaces() {
    uint8_t out[8] = {0};
    size_t len = 0;
    // A real ELM327 ignores whitespace anywhere in the command.
    TEST_ASSERT_TRUE(hexLineToBytes("  22 01\t01 ", out, sizeof(out), len));
    TEST_ASSERT_EQUAL_size_t(3, len);
    const uint8_t expected[] = {0x22, 0x01, 0x01};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, out, 3);
}

void test_accepts_lowercase_hex() {
    uint8_t out[8] = {0};
    size_t len = 0;
    TEST_ASSERT_TRUE(hexLineToBytes("2201ab", out, sizeof(out), len));
    TEST_ASSERT_EQUAL_size_t(3, len);
    TEST_ASSERT_EQUAL_UINT8(0xAB, out[2]);
}

void test_rejects_an_odd_digit_count() {
    uint8_t out[8] = {0};
    size_t len = 0;
    // Half a byte is not a request. Accepting it would silently drop or invent
    // a nibble and send a different PID than the client asked for.
    TEST_ASSERT_FALSE(hexLineToBytes("2201A", out, sizeof(out), len));
}

void test_rejects_non_hex_characters() {
    uint8_t out[8] = {0};
    size_t len = 0;
    TEST_ASSERT_FALSE(hexLineToBytes("2201ZZ", out, sizeof(out), len));
    TEST_ASSERT_FALSE(hexLineToBytes("hello", out, sizeof(out), len));
    TEST_ASSERT_FALSE(hexLineToBytes("22-01", out, sizeof(out), len));
}

void test_rejects_empty_input() {
    uint8_t out[8] = {0};
    size_t len = 0;
    TEST_ASSERT_FALSE(hexLineToBytes("", out, sizeof(out), len));
    TEST_ASSERT_EQUAL_size_t(0, len);
    // Whitespace only is empty too.
    TEST_ASSERT_FALSE(hexLineToBytes("   ", out, sizeof(out), len));
}

void test_rejects_a_line_longer_than_the_buffer_without_overflowing() {
    // 9 bytes into an 8-byte buffer, with a guard byte after it. If the bounds
    // check ran after the write instead of before, the guard would change.
    struct {
        uint8_t out[8];
        uint8_t guard;
    } framed = {};
    framed.guard = 0xA5;

    size_t len = 0;
    TEST_ASSERT_FALSE(hexLineToBytes("112233445566778899", framed.out,
                                     sizeof(framed.out), len));
    TEST_ASSERT_EQUAL_UINT8(0xA5, framed.guard);
}

// -------------------------------------------------------------- classifyResponse

void test_positive_response_echoes_the_service_plus_0x40() {
    const uint8_t request[] = {0x22, 0x01, 0x01};
    const uint8_t response[] = {0x62, 0x01, 0x01, 0xFF};
    TEST_ASSERT_EQUAL(ResponseMatch::Positive,
                      classifyResponse(request, sizeof(request),
                                       response, sizeof(response)));

    // Mode 01 is answered by 0x41.
    const uint8_t modeOneReq[] = {0x01, 0x0C};
    const uint8_t modeOneResp[] = {0x41, 0x0C, 0x1A, 0xF8};
    TEST_ASSERT_EQUAL(ResponseMatch::Positive,
                      classifyResponse(modeOneReq, sizeof(modeOneReq),
                                       modeOneResp, sizeof(modeOneResp)));
}

void test_answer_to_a_different_service_is_a_mismatch() {
    // THE bug this predicate exists to stop: a late reply to the PREVIOUS
    // request (mode 01) arriving while a mode 22 request is outstanding. Without
    // correlation this is decoded as the mode 22 payload and displayed as a
    // valid state-of-charge.
    const uint8_t request[] = {0x22, 0x01, 0x05};
    const uint8_t stale[] = {0x41, 0x0C, 0x1A, 0xF8};
    TEST_ASSERT_EQUAL(ResponseMatch::Mismatch,
                      classifyResponse(request, sizeof(request),
                                       stale, sizeof(stale)));
}

void test_response_pending_asks_the_caller_to_keep_waiting() {
    const uint8_t request[] = {0x22, 0x01, 0x01};
    // 7F <service> 78 = requestCorrectlyReceived-ResponsePending.
    const uint8_t pending[] = {0x7F, 0x22, 0x78};
    TEST_ASSERT_EQUAL(ResponseMatch::NegativePending,
                      classifyResponse(request, sizeof(request),
                                       pending, sizeof(pending)));
}

void test_other_negative_responses_are_final_rejections() {
    const uint8_t request[] = {0x22, 0x01, 0x01};
    const uint8_t notSupported[] = {0x7F, 0x22, 0x11};   // serviceNotSupported
    const uint8_t outOfRange[]  = {0x7F, 0x22, 0x31};    // requestOutOfRange
    const uint8_t conditions[]  = {0x7F, 0x22, 0x22};    // conditionsNotCorrect
    TEST_ASSERT_EQUAL(ResponseMatch::NegativeFinal,
                      classifyResponse(request, sizeof(request),
                                       notSupported, sizeof(notSupported)));
    TEST_ASSERT_EQUAL(ResponseMatch::NegativeFinal,
                      classifyResponse(request, sizeof(request),
                                       outOfRange, sizeof(outOfRange)));
    TEST_ASSERT_EQUAL(ResponseMatch::NegativeFinal,
                      classifyResponse(request, sizeof(request),
                                       conditions, sizeof(conditions)));
}

void test_negative_response_to_another_service_is_a_mismatch() {
    // 0x7F carries the service it is rejecting. If that isn't ours, the
    // rejection belongs to someone else's request and must not end ours.
    const uint8_t request[] = {0x22, 0x01, 0x01};
    const uint8_t theirs[] = {0x7F, 0x19, 0x11};
    TEST_ASSERT_EQUAL(ResponseMatch::Mismatch,
                      classifyResponse(request, sizeof(request),
                                       theirs, sizeof(theirs)));
}

void test_truncated_negative_response_is_a_mismatch_not_a_rejection() {
    const uint8_t request[] = {0x22, 0x01, 0x01};
    const uint8_t truncated[] = {0x7F, 0x22};
    TEST_ASSERT_EQUAL(ResponseMatch::Mismatch,
                      classifyResponse(request, sizeof(request),
                                       truncated, sizeof(truncated)));
}

void test_empty_or_null_inputs_are_a_mismatch() {
    const uint8_t request[] = {0x22, 0x01, 0x01};
    const uint8_t response[] = {0x62, 0x01, 0x01};
    TEST_ASSERT_EQUAL(ResponseMatch::Mismatch,
                      classifyResponse(request, sizeof(request), response, 0));
    TEST_ASSERT_EQUAL(ResponseMatch::Mismatch,
                      classifyResponse(request, 0, response, sizeof(response)));
    TEST_ASSERT_EQUAL(ResponseMatch::Mismatch,
                      classifyResponse(nullptr, 1, response, sizeof(response)));
}

void test_service_echo_wraps_without_overflow() {
    // 0xC0 + 0x40 == 0x00 in a uint8_t. The comparison must be done in the same
    // width the wire uses, or a 0xC0 request would never match anything.
    const uint8_t request[] = {0xC0, 0x01};
    const uint8_t response[] = {0x00, 0x01};
    TEST_ASSERT_EQUAL(ResponseMatch::Positive,
                      classifyResponse(request, sizeof(request),
                                       response, sizeof(response)));
}

void test_did_match_is_required_for_service_22() {
    const uint8_t request[] = {0x22, 0x01, 0x01};
    const uint8_t response[] = {0x62, 0x01, 0x01, 0xFF};
    TEST_ASSERT_EQUAL(ResponseMatch::Positive,
                      classifyResponse(request, sizeof(request),
                                       response, sizeof(response)));
}

void test_wrong_did_same_service_is_a_mismatch() {
    // THE actual bug: a late reply to 220105 arriving during the 220101
    // receive window. Same service (0x62), wrong DID. Must not pass.
    const uint8_t request[] = {0x22, 0x01, 0x01};
    const uint8_t wrongDid[] = {0x62, 0x01, 0x05, 0xFF};
    TEST_ASSERT_EQUAL(ResponseMatch::Mismatch,
                      classifyResponse(request, sizeof(request),
                                       wrongDid, sizeof(wrongDid)));
}

void test_matching_did_for_a_different_pid_is_positive() {
    const uint8_t request[] = {0x22, 0x01, 0x05};
    const uint8_t response[] = {0x62, 0x01, 0x05, 0x2A};
    TEST_ASSERT_EQUAL(ResponseMatch::Positive,
                      classifyResponse(request, sizeof(request),
                                       response, sizeof(response)));
}

void test_mode_01_pid_echo_is_verified() {
    const uint8_t request[] = {0x01, 0x00};
    const uint8_t okResponse[] = {0x41, 0x00, 0xBE, 0x3F, 0xA8, 0x13};
    TEST_ASSERT_EQUAL(ResponseMatch::Positive,
                      classifyResponse(request, sizeof(request),
                                       okResponse, sizeof(okResponse)));

    // Right service (0x41), wrong PID.
    const uint8_t wrongPid[] = {0x41, 0x0C, 0x1A, 0xF8};
    TEST_ASSERT_EQUAL(ResponseMatch::Mismatch,
                      classifyResponse(request, sizeof(request),
                                       wrongPid, sizeof(wrongPid)));
}

void test_service_with_no_verified_echo_shape_stays_service_only() {
    // The owner deliberately keeps write/control services (0x2E here) passing
    // on service match alone; this project must not invent an echo rule for a
    // service it cannot verify.
    const uint8_t request[] = {0x2E, 0xF1, 0x90, 0x01, 0x02};
    const uint8_t response[] = {0x6E, 0xAA, 0xBB};
    TEST_ASSERT_EQUAL(ResponseMatch::Positive,
                      classifyResponse(request, sizeof(request),
                                       response, sizeof(response)));
}

void test_truncated_response_too_short_for_did_comparison_is_a_mismatch() {
    // Right service byte, but too short to contain the DID bytes needed for
    // the comparison. Must not read past the end of the buffer.
    const uint8_t request[] = {0x22, 0x01, 0x01};
    const uint8_t truncated[] = {0x62};
    TEST_ASSERT_EQUAL(ResponseMatch::Mismatch,
                      classifyResponse(request, sizeof(request),
                                       truncated, sizeof(truncated)));

    const uint8_t truncated2[] = {0x62, 0x01};
    TEST_ASSERT_EQUAL(ResponseMatch::Mismatch,
                      classifyResponse(request, sizeof(request),
                                       truncated2, sizeof(truncated2)));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_parses_a_plain_hex_line);
    RUN_TEST(test_tolerates_embedded_spaces);
    RUN_TEST(test_accepts_lowercase_hex);
    RUN_TEST(test_rejects_an_odd_digit_count);
    RUN_TEST(test_rejects_non_hex_characters);
    RUN_TEST(test_rejects_empty_input);
    RUN_TEST(test_rejects_a_line_longer_than_the_buffer_without_overflowing);
    RUN_TEST(test_positive_response_echoes_the_service_plus_0x40);
    RUN_TEST(test_answer_to_a_different_service_is_a_mismatch);
    RUN_TEST(test_response_pending_asks_the_caller_to_keep_waiting);
    RUN_TEST(test_other_negative_responses_are_final_rejections);
    RUN_TEST(test_negative_response_to_another_service_is_a_mismatch);
    RUN_TEST(test_truncated_negative_response_is_a_mismatch_not_a_rejection);
    RUN_TEST(test_empty_or_null_inputs_are_a_mismatch);
    RUN_TEST(test_service_echo_wraps_without_overflow);
    RUN_TEST(test_did_match_is_required_for_service_22);
    RUN_TEST(test_wrong_did_same_service_is_a_mismatch);
    RUN_TEST(test_matching_did_for_a_different_pid_is_positive);
    RUN_TEST(test_mode_01_pid_echo_is_verified);
    RUN_TEST(test_service_with_no_verified_echo_shape_stays_service_only);
    RUN_TEST(test_truncated_response_too_short_for_did_comparison_is_a_mismatch);
    return UNITY_END();
}
