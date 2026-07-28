#include "can/can_bus.h"
#include "board_pins.h"
#include <Arduino.h>

static CanCounters g_counters;

bool canBusBegin(bool listenOnly) {
    // Drive CAN_RS low for high-speed normal mode. Floating selects
    // slope-control, which mangles 500 kbit/s frames.
    pinMode(CAN_RS_PIN, OUTPUT);
    digitalWrite(CAN_RS_PIN, LOW);

    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_RX_PIN,
        listenOnly ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
    general.rx_queue_len = 64;
    general.tx_queue_len = 16;

    twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&general, &timing, &filter) != ESP_OK) return false;
    return twai_start() == ESP_OK;
}

bool canBusReceive(twai_message_t& out, uint32_t timeoutMs) {
    if (twai_receive(&out, pdMS_TO_TICKS(timeoutMs)) != ESP_OK) return false;
    g_counters.received++;
    return true;
}

namespace {

// Called only when a transmit has just failed — the one path that observes the
// problem. Transmitting to a sleeping car produces unacked frames; the transmit
// error counter climbs and the controller latches into bus-off, after which
// every request returns CAN ERROR until a power cycle. The board is
// ignition-powered and holds itself on via FORCE_ON, so an ordinary sleep/wake
// cycle reaches this state.
//
// Deliberately non-blocking. Bus-off recovery is asynchronous: the controller
// must observe 128 occurrences of 11 consecutive recessive bits before it is
// allowed back on, which cannot be waited out inside a 200 ms request budget
// and must not block the BLE host task. So this kicks recovery off and returns;
// the current request still answers CAN ERROR, and a later request finds the
// driver STOPPED (recovery complete) and restarts it. The link heals itself
// over the next poll or two without a reboot.
void recoverIfBusOff() {
    twai_status_info_t status;
    if (twai_get_status_info(&status) != ESP_OK) return;

    if (status.state == TWAI_STATE_BUS_OFF) {
        twai_initiate_recovery();
    } else if (status.state == TWAI_STATE_STOPPED) {
        // Recovery finished (or the driver was stopped some other way): the
        // TWAI driver leaves the controller stopped and waits to be restarted.
        twai_start();
    }
}

} // namespace

bool canBusTransmit(const twai_message_t& msg, uint32_t timeoutMs) {
    if (twai_transmit(&msg, pdMS_TO_TICKS(timeoutMs)) == ESP_OK) return true;
    recoverIfBusOff();
    return false;
}

CanCounters canBusCounters() {
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        g_counters.busErrors = status.bus_error_count;
        g_counters.rxMissed = status.rx_missed_count;
        g_counters.rxOverrun = status.rx_overrun_count;
    }
    return g_counters;
}
