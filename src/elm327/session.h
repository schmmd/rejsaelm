#pragma once
#include <string>
#include "elm327/at_parser.h"

// One ELM327 conversation. Strictly half-duplex: handleLine() runs a complete
// request/response exchange before returning, so a client that respects the
// prompt can never have two commands in flight.
class Elm327Session {
public:
    // Returns the full reply text INCLUDING the trailing prompt.
    std::string handleLine(const char* line);

private:
    std::string runRequest(const char* hexLine);
    AdapterState state_;
};
