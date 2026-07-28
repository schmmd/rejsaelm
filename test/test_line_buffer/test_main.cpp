#include <unity.h>
#include <cstring>
#include "elm327/line_buffer.h"

void setUp() {}
void tearDown() {}

void test_accumulates_until_cr() {
    LineBuffer b;
    TEST_ASSERT_FALSE(b.offer('A'));
    TEST_ASSERT_FALSE(b.offer('T'));
    TEST_ASSERT_FALSE(b.offer('I'));
    TEST_ASSERT_TRUE(b.offer('\r'));
    TEST_ASSERT_EQUAL_STRING("ATI", b.line());
}

void test_resets_between_lines() {
    LineBuffer b;
    for (const char* p = "ATI\r"; *p; ++p) b.offer(*p);
    for (const char* p = "ATZ\r"; *p; ++p) b.offer(*p);
    TEST_ASSERT_EQUAL_STRING("ATZ", b.line());
}

void test_empty_line_is_complete_but_empty() {
    // A bare CR is what a client sends to repeat the last command on a real
    // ELM327. This firmware has no repeat feature, so the caller will answer
    // "?" — but the buffer must still report a completed, empty line rather
    // than swallowing the CR.
    LineBuffer b;
    TEST_ASSERT_TRUE(b.offer('\r'));
    TEST_ASSERT_EQUAL_STRING("", b.line());
}

void test_linefeeds_and_spaces_are_ignored() {
    // Terminals and scripts send CRLF; ELM327 clients pad with spaces.
    LineBuffer b;
    for (const char* p = "22 01 01"; *p; ++p) b.offer(*p);
    TEST_ASSERT_FALSE(b.offer('\n'));
    TEST_ASSERT_TRUE(b.offer('\r'));
    TEST_ASSERT_EQUAL_STRING("220101", b.line());
}

void test_overlong_input_is_discarded_to_next_cr() {
    // A 64-byte buffer cannot hold a runaway line. Report the overflow once,
    // at the CR, with an empty line so the caller answers "?" — never a
    // truncated command, which would send a DIFFERENT request than was asked
    // for and record its answer under the wrong DID.
    LineBuffer b;
    for (int i = 0; i < 200; ++i) TEST_ASSERT_FALSE(b.offer('A'));
    TEST_ASSERT_TRUE(b.offer('\r'));
    TEST_ASSERT_TRUE(b.overflowed());
    TEST_ASSERT_EQUAL_STRING("", b.line());
}

void test_recovers_after_overflow() {
    LineBuffer b;
    for (int i = 0; i < 200; ++i) b.offer('A');
    b.offer('\r');
    for (const char* p = "ATI\r"; *p; ++p) b.offer(*p);
    TEST_ASSERT_FALSE(b.overflowed());
    TEST_ASSERT_EQUAL_STRING("ATI", b.line());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_accumulates_until_cr);
    RUN_TEST(test_resets_between_lines);
    RUN_TEST(test_empty_line_is_complete_but_empty);
    RUN_TEST(test_linefeeds_and_spaces_are_ignored);
    RUN_TEST(test_overlong_input_is_discarded_to_next_cr);
    RUN_TEST(test_recovers_after_overflow);
    return UNITY_END();
}
