#include "isotp/reassembler.h"

void IsoTpReassembler::reset() {
    payload_.clear();
    expected_ = 0;
    nextSequence_ = 0;
    collecting_ = false;
}

IsoTpState IsoTpReassembler::offer(const uint8_t* frame, size_t len) {
    if (len < 1) return IsoTpState::Error;

    // len comes from a hardware DLC register on a bus we don't control; a
    // malformed frame can report up to 15. Clamp to the real max frame size
    // (8) so a bogus DLC can never make us read past the caller's buffer.
    if (len > 8) len = 8;

    const uint8_t type = frame[0] >> 4;

    if (type == 0x0) {  // single frame
        const size_t length = frame[0] & 0x0F;
        if (length == 0 || length + 1 > len) return IsoTpState::Error;
        reset();
        payload_.assign(frame + 1, frame + 1 + length);
        return IsoTpState::Complete;
    }

    if (type == 0x1) {  // first frame
        if (len < 2) return IsoTpState::Error;
        reset();
        expected_ = ((static_cast<size_t>(frame[0]) & 0x0F) << 8) | frame[1];
        if (expected_ == 0) return IsoTpState::Error;
        const size_t take = (len - 2 < expected_) ? len - 2 : expected_;
        payload_.assign(frame + 2, frame + 2 + take);
        nextSequence_ = 1;
        collecting_ = true;
        return IsoTpState::NeedFlowControl;
    }

    if (type == 0x2) {  // consecutive frame
        if (!collecting_) return IsoTpState::Error;
        const uint8_t sequence = frame[0] & 0x0F;
        if (sequence != nextSequence_) return IsoTpState::Error;
        nextSequence_ = (nextSequence_ + 1) & 0x0F;

        const size_t remaining = expected_ - payload_.size();
        const size_t available = len - 1;
        const size_t take = (available < remaining) ? available : remaining;
        payload_.insert(payload_.end(), frame + 1, frame + 1 + take);

        if (payload_.size() >= expected_) {
            collecting_ = false;
            return IsoTpState::Complete;
        }
        return IsoTpState::Collecting;
    }

    // Flow-control frames from the ECU are not part of a response we collect.
    return IsoTpState::Error;
}
