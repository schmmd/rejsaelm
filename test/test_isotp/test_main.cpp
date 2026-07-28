#include <unity.h>
#include "isotp/reassembler.h"
#include "isotp/request.h"

void setUp() {}
void tearDown() {}

void test_single_frame_completes_immediately() {
    IsoTpReassembler r;
    const uint8_t frame[8] = {0x03, 0x41, 0x00, 0xBE, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL(IsoTpState::Complete, r.offer(frame, 8));
    TEST_ASSERT_EQUAL_UINT(3, r.payload().size());
    TEST_ASSERT_EQUAL_UINT8(0x41, r.payload()[0]);
    TEST_ASSERT_EQUAL_UINT8(0xBE, r.payload()[2]);
}

void test_single_frame_with_zero_length_is_an_error() {
    IsoTpReassembler r;
    const uint8_t frame[8] = {0x00, 0, 0, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL(IsoTpState::Error, r.offer(frame, 8));
}

void test_first_frame_requests_flow_control() {
    IsoTpReassembler r;
    // 0x10 0x14 = first frame, total length 0x014 = 20 bytes.
    const uint8_t ff[8] = {0x10, 0x14, 0x62, 0x01, 0x01, 0xFF, 0xF7, 0xE7};
    TEST_ASSERT_EQUAL(IsoTpState::NeedFlowControl, r.offer(ff, 8));
    TEST_ASSERT_EQUAL_UINT(6, r.payload().size());  // 6 data bytes in a FF
}

void test_consecutive_frames_complete_the_payload() {
    IsoTpReassembler r;
    const uint8_t ff[8] = {0x10, 0x14, 0x62, 0x01, 0x01, 0xFF, 0xF7, 0xE7};
    TEST_ASSERT_EQUAL(IsoTpState::NeedFlowControl, r.offer(ff, 8));

    const uint8_t cf1[8] = {0x21, 0xFF, 0x8C, 0x00, 0x4B, 0x00, 0x00, 0x00};
    TEST_ASSERT_EQUAL(IsoTpState::Collecting, r.offer(cf1, 8));

    const uint8_t cf2[8] = {0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    TEST_ASSERT_EQUAL(IsoTpState::Complete, r.offer(cf2, 8));

    TEST_ASSERT_EQUAL_UINT(20, r.payload().size());
    TEST_ASSERT_EQUAL_UINT8(0x62, r.payload()[0]);
    TEST_ASSERT_EQUAL_UINT8(0x4B, r.payload()[9]);
}

void test_out_of_order_sequence_number_is_an_error() {
    IsoTpReassembler r;
    const uint8_t ff[8] = {0x10, 0x14, 0x62, 0x01, 0x01, 0xFF, 0xF7, 0xE7};
    r.offer(ff, 8);
    // Expecting sequence 1, receiving 2.
    const uint8_t cf[8] = {0x22, 0xFF, 0x8C, 0x00, 0x4B, 0x00, 0x00, 0x00};
    TEST_ASSERT_EQUAL(IsoTpState::Error, r.offer(cf, 8));
}

void test_long_response_with_truncated_final_frame() {
    IsoTpReassembler r;
    // 100 bytes: FF carries 6, then 14 consecutive frames of 7 = 98 more, with final frame truncated.
    uint8_t ff[8] = {0x10, 0x64, 1, 2, 3, 4, 5, 6};
    TEST_ASSERT_EQUAL(IsoTpState::NeedFlowControl, r.offer(ff, 8));

    IsoTpState last = IsoTpState::Collecting;
    for (int i = 0; i < 14; ++i) {
        uint8_t cf[8] = {static_cast<uint8_t>(0x20 | ((i + 1) & 0x0F)), 0, 0, 0, 0, 0, 0, 0};
        last = r.offer(cf, 8);
        if (last == IsoTpState::Error) break;
    }
    TEST_ASSERT_NOT_EQUAL(IsoTpState::Error, last);
    TEST_ASSERT_EQUAL(IsoTpState::Complete, last);
    TEST_ASSERT_EQUAL_UINT(100, r.payload().size());
}

void test_sequence_number_wraps_past_fifteen() {
    IsoTpReassembler r;
    // 125 bytes: FF carries 6, then 17 consecutive frames of 7 = 119 more.
    // Sequences: 1,2,...,15,0,1 — ensures counter wraps 15→0 and continues correctly.
    uint8_t ff[8] = {0x10, 0x7D, 1, 2, 3, 4, 5, 6};
    TEST_ASSERT_EQUAL(IsoTpState::NeedFlowControl, r.offer(ff, 8));

    IsoTpState last = IsoTpState::Collecting;
    for (int i = 0; i < 17; ++i) {
        uint8_t cf[8] = {static_cast<uint8_t>(0x20 | ((i + 1) & 0x0F)), 0, 0, 0, 0, 0, 0, 0};
        last = r.offer(cf, 8);
        if (last == IsoTpState::Error) break;
    }
    TEST_ASSERT_NOT_EQUAL(IsoTpState::Error, last);
    TEST_ASSERT_EQUAL(IsoTpState::Complete, last);
    TEST_ASSERT_EQUAL_UINT(125, r.payload().size());
}

void test_consecutive_frame_without_first_frame_is_an_error() {
    IsoTpReassembler r;
    const uint8_t cf[8] = {0x21, 1, 2, 3, 4, 5, 6, 7};
    TEST_ASSERT_EQUAL(IsoTpState::Error, r.offer(cf, 8));
}

void test_reset_clears_state() {
    IsoTpReassembler r;
    const uint8_t ff[8] = {0x10, 0x14, 0x62, 0x01, 0x01, 0xFF, 0xF7, 0xE7};
    r.offer(ff, 8);
    r.reset();
    TEST_ASSERT_EQUAL_UINT(0, r.payload().size());
    const uint8_t cf[8] = {0x21, 1, 2, 3, 4, 5, 6, 7};
    TEST_ASSERT_EQUAL(IsoTpState::Error, r.offer(cf, 8));
}

void test_flow_control_frame_is_an_error() {
    IsoTpReassembler r;
    // Flow-control frame: PCI type 3 (first byte 0x30)
    const uint8_t fc[8] = {0x30, 0, 0, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL(IsoTpState::Error, r.offer(fc, 8));
}

void test_single_frame_request_is_length_prefixed_and_padded() {
    // "220101" as bytes = 22 01 01, a 3-byte request.
    const uint8_t payload[] = {0x22, 0x01, 0x01};
    uint8_t frame[8] = {0};
    TEST_ASSERT_TRUE(buildSingleFrameRequest(payload, sizeof(payload), frame));
    TEST_ASSERT_EQUAL_UINT8(0x03, frame[0]);   // PCI: single frame, length 3
    TEST_ASSERT_EQUAL_UINT8(0x22, frame[1]);
    TEST_ASSERT_EQUAL_UINT8(0x01, frame[2]);
    TEST_ASSERT_EQUAL_UINT8(0x01, frame[3]);
    // Remainder padded with 0x00 so the frame is always 8 bytes.
    TEST_ASSERT_EQUAL_UINT8(0x00, frame[4]);
    TEST_ASSERT_EQUAL_UINT8(0x00, frame[7]);
}

void test_request_longer_than_seven_bytes_is_rejected() {
    const uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t frame[8] = {0};
    // Multi-frame REQUESTS are not needed: every request this firmware serves
    // fits one frame. Reject rather than silently truncate.
    TEST_ASSERT_FALSE(buildSingleFrameRequest(payload, sizeof(payload), frame));
}

void test_empty_request_is_rejected() {
    uint8_t frame[8] = {0};
    TEST_ASSERT_FALSE(buildSingleFrameRequest(nullptr, 0, frame));
}

void test_flow_control_frame_grants_everything_with_no_delay() {
    uint8_t frame[8] = {0};
    buildFlowControlFrame(frame);
    TEST_ASSERT_EQUAL_UINT8(0x30, frame[0]);  // FC, ContinueToSend
    TEST_ASSERT_EQUAL_UINT8(0x00, frame[1]);  // block size 0 = send everything
    TEST_ASSERT_EQUAL_UINT8(0x00, frame[2]);  // STmin 0 = no inter-frame delay
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_single_frame_completes_immediately);
    RUN_TEST(test_single_frame_with_zero_length_is_an_error);
    RUN_TEST(test_first_frame_requests_flow_control);
    RUN_TEST(test_consecutive_frames_complete_the_payload);
    RUN_TEST(test_out_of_order_sequence_number_is_an_error);
    RUN_TEST(test_long_response_with_truncated_final_frame);
    RUN_TEST(test_sequence_number_wraps_past_fifteen);
    RUN_TEST(test_consecutive_frame_without_first_frame_is_an_error);
    RUN_TEST(test_reset_clears_state);
    RUN_TEST(test_flow_control_frame_is_an_error);
    RUN_TEST(test_single_frame_request_is_length_prefixed_and_padded);
    RUN_TEST(test_request_longer_than_seven_bytes_is_rejected);
    RUN_TEST(test_empty_request_is_rejected);
    RUN_TEST(test_flow_control_frame_grants_everything_with_no_delay);
    return UNITY_END();
}
