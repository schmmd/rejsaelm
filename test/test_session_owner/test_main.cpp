#include <unity.h>
#include "link/session_owner.h"

void setUp() { sessionOwner().store(LinkId::None); }
void tearDown() {}

void test_first_claim_wins() {
    TEST_ASSERT_TRUE(sessionOwnerClaim(LinkId::Ble));
    TEST_ASSERT_TRUE(sessionOwnerIs(LinkId::Ble));
}

void test_other_transport_is_rejected_while_held() {
    // The whole point: BLE and WiFi are both up, and the second one to arrive
    // must not get the session. Its client is disconnected instead.
    TEST_ASSERT_TRUE(sessionOwnerClaim(LinkId::Ble));
    TEST_ASSERT_FALSE(sessionOwnerClaim(LinkId::Wifi));
    TEST_ASSERT_TRUE(sessionOwnerIs(LinkId::Ble));
}

void test_second_client_on_the_same_transport_is_rejected() {
    // Same call also does the within-transport arbitration BLE used to do with
    // its own flag, so there is one rule rather than two that can disagree.
    TEST_ASSERT_TRUE(sessionOwnerClaim(LinkId::Wifi));
    TEST_ASSERT_FALSE(sessionOwnerClaim(LinkId::Wifi));
}

void test_release_frees_it_for_the_other_transport() {
    sessionOwnerClaim(LinkId::Ble);
    sessionOwnerRelease(LinkId::Ble);
    TEST_ASSERT_TRUE(sessionOwnerIs(LinkId::None));
    TEST_ASSERT_TRUE(sessionOwnerClaim(LinkId::Wifi));
}

void test_rejected_transport_cannot_release_the_owner() {
    // The regression this file exists for. Rejecting a client means
    // disconnecting it, which runs the same teardown as a real departure. If
    // that teardown released unconditionally, the rejected WiFi client would
    // hand the session away from the live BLE owner -- and a bench test would
    // still show the newcomer being dropped, so it would look correct.
    sessionOwnerClaim(LinkId::Ble);
    sessionOwnerClaim(LinkId::Wifi);   // rejected
    sessionOwnerRelease(LinkId::Wifi); // its socket closes, running teardown
    TEST_ASSERT_TRUE(sessionOwnerIs(LinkId::Ble));
}

void test_release_when_nobody_holds_it_is_harmless() {
    sessionOwnerRelease(LinkId::Wifi);
    TEST_ASSERT_TRUE(sessionOwnerIs(LinkId::None));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_first_claim_wins);
    RUN_TEST(test_other_transport_is_rejected_while_held);
    RUN_TEST(test_second_client_on_the_same_transport_is_rejected);
    RUN_TEST(test_release_frees_it_for_the_other_transport);
    RUN_TEST(test_rejected_transport_cannot_release_the_owner);
    RUN_TEST(test_release_when_nobody_holds_it_is_harmless);
    return UNITY_END();
}
