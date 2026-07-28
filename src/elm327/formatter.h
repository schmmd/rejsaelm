#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include "elm327/at_parser.h"

// End-of-reply marker. The client uses this to know a command completed.
extern const char* kPrompt;

// Renders a reassembled payload in the ELM327 headers-off form.
//
// SHARED CONTRACT: this output is parsed by
// android/elm327/src/main/kotlin/nu/ioniqcan/elm327/ResponseParser.kt.
// The fixtures in test_formatter/ come from that module's tests. Changing the
// format here without changing it there breaks the app silently — decoded
// values shift rather than erroring out.
std::string formatPayload(const uint8_t* data, size_t len, const AdapterState& state);

std::string formatFault(const char* faultText);

// NRC reporting belongs to the scan build and nothing else.
//
// -DREPORT_NRC and -DUSB_SERIAL_LINK are independent build flags that today
// happen to be set together by exactly one env (rejsacan_usbscan). Nothing
// stopped a future env from enabling NRC reporting on a BLE build, and nothing
// would have failed if it did: session.cpp's two NegativeFinal branches are
// excluded from env:native (the file needs Arduino.h and the TWAI driver), so
// no host test covers either one. This is the cheapest place to make the
// combination impossible instead of merely unintended — a header every
// translation unit that can format an NRC must include, in every env.
#if defined(REPORT_NRC) && !defined(USB_SERIAL_LINK)
#error "REPORT_NRC is scan-build only; the Android app must keep seeing NO DATA"
#endif

// Renders a negative-response code as "NRC xx".
//
// Only the scan build (-DREPORT_NRC) ever emits this. The shipping firmware
// answers a final negative with formatFault("NO DATA") — see session.cpp. The
// tag keeps it unambiguous against both the fault words and a hex payload line,
// which is what stops ResponseParser.kt from ever turning this string into
// bytes; test_formatter pins that property for every byte value.
std::string formatNrc(uint8_t nrc);
