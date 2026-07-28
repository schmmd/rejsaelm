#include <unity.h>
#include <cstddef>
#include <string>
#include <vector>

#include "link/write_all.h"

void setUp() {}
void tearDown() {}

namespace {

// A sink that accepts at most `chunk` bytes per call, the way USB CDC does
// when its TX FIFO is nearly full. `zeros` calls return 0 first, standing in
// for a FIFO with no room at all and a host that is not draining it.
struct PartialSink {
    size_t chunk;
    int zeros = 0;
    std::string written;
    int calls = 0;

    size_t operator()(const char* data, size_t size) {
        ++calls;
        if (zeros > 0) {
            --zeros;
            return 0;
        }
        const size_t n = size < chunk ? size : chunk;
        written.append(data, n);
        return n;
    }
};

} // namespace

void test_a_sink_that_takes_everything_is_written_once() {
    PartialSink sink{64};
    const std::string reply = "ELM327 v1.5\r>";
    TEST_ASSERT_EQUAL_UINT(reply.size(),
                           writeAll(sink, reply.data(), reply.size()));
    TEST_ASSERT_EQUAL_STRING(reply.c_str(), sink.written.c_str());
    TEST_ASSERT_EQUAL_INT(1, sink.calls);
}

void test_a_short_write_is_continued_until_the_whole_reply_is_out() {
    // THE BUG. USBCDC::write returns a SHORT COUNT when the TX FIFO fills and
    // its internal timeout expires; it does not promise to send everything.
    // serial_link.cpp ignored the return value, so the remainder -- including
    // the ">" prompt the host reads to know the reply ended -- was dropped.
    // On the car this surfaced as the scanner reading b'NRC 3': five bytes of
    // "NRC 31\r>" arrived and the rest never did.
    PartialSink sink{5};
    const std::string reply = "NRC 31\r>";
    TEST_ASSERT_EQUAL_UINT(reply.size(),
                           writeAll(sink, reply.data(), reply.size()));
    TEST_ASSERT_EQUAL_STRING(reply.c_str(), sink.written.c_str());
    TEST_ASSERT_TRUE(sink.calls > 1);
}

void test_a_sink_that_takes_one_byte_at_a_time_still_completes() {
    PartialSink sink{1};
    const std::string reply = "620101AA\r>";
    TEST_ASSERT_EQUAL_UINT(reply.size(),
                           writeAll(sink, reply.data(), reply.size()));
    TEST_ASSERT_EQUAL_STRING(reply.c_str(), sink.written.c_str());
}

void test_a_transient_zero_write_is_retried_rather_than_abandoned() {
    // A full FIFO is the normal case this exists to survive: zero written now
    // does not mean zero writable a moment later.
    PartialSink sink{64};
    sink.zeros = 2;
    const std::string reply = "OK\r>";
    TEST_ASSERT_EQUAL_UINT(reply.size(),
                           writeAll(sink, reply.data(), reply.size()));
    TEST_ASSERT_EQUAL_STRING(reply.c_str(), sink.written.c_str());
}

void test_a_sink_that_never_accepts_gives_up_rather_than_spinning() {
    // The other half of the contract. serialLinkPoll() runs in loop(); a sink
    // that never drains -- an unplugged host holding CDC open -- must not
    // wedge the firmware in an unbounded retry. Report what got out and let
    // the caller carry on; the client sees a gapped reply and times out, which
    // is recoverable, whereas a hung loop() is not.
    PartialSink sink{64};
    sink.zeros = 1000000;
    const std::string reply = "OK\r>";
    TEST_ASSERT_EQUAL_UINT(0, writeAll(sink, reply.data(), reply.size()));
    TEST_ASSERT_TRUE(sink.calls <= kWriteAllMaxStalledAttempts);
}

void test_a_partial_write_followed_by_a_dead_sink_reports_what_got_out() {
    PartialSink sink{3};
    sink.written.clear();
    const std::string reply = "620101AA\r>";
    // Three bytes go, then the sink dies for good.
    const size_t first = writeAll(sink, reply.data(), 3);
    TEST_ASSERT_EQUAL_UINT(3, first);
    sink.zeros = 1000000;
    TEST_ASSERT_EQUAL_UINT(0, writeAll(sink, reply.data() + 3, 3));
}

void test_an_empty_reply_writes_nothing_and_calls_nothing() {
    PartialSink sink{64};
    TEST_ASSERT_EQUAL_UINT(0, writeAll(sink, "", 0));
    TEST_ASSERT_EQUAL_INT(0, sink.calls);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_a_sink_that_takes_everything_is_written_once);
    RUN_TEST(test_a_short_write_is_continued_until_the_whole_reply_is_out);
    RUN_TEST(test_a_sink_that_takes_one_byte_at_a_time_still_completes);
    RUN_TEST(test_a_transient_zero_write_is_retried_rather_than_abandoned);
    RUN_TEST(test_a_sink_that_never_accepts_gives_up_rather_than_spinning);
    RUN_TEST(test_a_partial_write_followed_by_a_dead_sink_reports_what_got_out);
    RUN_TEST(test_an_empty_reply_writes_nothing_and_calls_nothing);
    return UNITY_END();
}
