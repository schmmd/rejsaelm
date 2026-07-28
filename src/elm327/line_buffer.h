#pragma once
#include <cstddef>

// Accumulates incoming characters into one CR-terminated command line.
//
// Mirrors what the BLE link does implicitly (a GATT write arrives as a whole
// line); a byte-at-a-time serial port has to be told where a line ends.
//
// Linefeeds and spaces are dropped, so CRLF terminals and space-padded hex
// both work. Input longer than the buffer sets the overflow flag and yields an
// EMPTY line at the next CR: a truncated command would be a syntactically
// valid but DIFFERENT request, and its answer would be filed under the wrong
// identifier.
class LineBuffer {
public:
    // Returns true when `c` completed a line.
    bool offer(char c);

    // Valid until the next offer() that returns true.
    const char* line() const { return line_; }

    // True if the line just completed was discarded for being too long.
    bool overflowed() const { return overflowed_; }

    void reset();

private:
    static constexpr size_t kMaxLine = 64;

    char line_[kMaxLine + 1] = {0};
    size_t len_ = 0;
    bool overflowed_ = false;
    bool discarding_ = false;
};
