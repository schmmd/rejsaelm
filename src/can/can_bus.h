#pragma once
#include <driver/twai.h>
#include <cstdint>

struct CanCounters {
    uint32_t received = 0;
    uint32_t busErrors = 0;
    uint32_t rxMissed = 0;
    uint32_t rxOverrun = 0;
};

// listenOnly=true puts the controller in TWAI_MODE_LISTEN_ONLY: it never
// drives the bus, sends no ACKs, and emits no error frames. Use it for the
// bus test. The ELM327 build must pass false — answering a diagnostic request
// requires transmitting.
bool canBusBegin(bool listenOnly);

bool canBusReceive(twai_message_t& out, uint32_t timeoutMs);
bool canBusTransmit(const twai_message_t& msg, uint32_t timeoutMs);
CanCounters canBusCounters();
