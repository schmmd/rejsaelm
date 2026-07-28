#include "elm327/line_buffer.h"

void LineBuffer::reset() {
    len_ = 0;
    line_[0] = '\0';
    overflowed_ = false;
    discarding_ = false;
}

bool LineBuffer::offer(char c) {
    if (c == '\n' || c == ' ') return false;

    if (c == '\r') {
        overflowed_ = discarding_;
        if (discarding_) {
            len_ = 0;
        }
        line_[len_] = '\0';
        len_ = 0;
        discarding_ = false;
        return true;
    }

    if (discarding_) return false;

    if (len_ >= kMaxLine) {
        discarding_ = true;
        return false;
    }

    line_[len_++] = c;
    return false;
}
