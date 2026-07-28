#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

enum class IsoTpState {
    Idle,
    NeedFlowControl,  // first frame seen; caller must send a flow-control frame
    Collecting,
    Complete,
    Error,
};

// Collects an ISO 15765-2 response. One instance handles one response at a
// time; call reset() between requests.
class IsoTpReassembler {
public:
    IsoTpState offer(const uint8_t* frame, size_t len);
    const std::vector<uint8_t>& payload() const { return payload_; }
    void reset();

private:
    std::vector<uint8_t> payload_;
    size_t expected_ = 0;
    uint8_t nextSequence_ = 0;
    bool collecting_ = false;
};
