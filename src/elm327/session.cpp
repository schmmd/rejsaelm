#include "elm327/session.h"
#include "elm327/formatter.h"
#include "elm327/hexline.h"
#include "can/can_bus.h"
#include "isotp/reassembler.h"
#include "isotp/request.h"
#include <Arduino.h>
#include <cstring>

namespace {

const char* kVersionBanner = "ELM327 v1.5";
// @1 — a device description, distinct from the ATI version banner.
const char* kDeviceDescription = "RejsaElm OBD-II Adapter";

} // namespace

std::string Elm327Session::handleLine(const char* line) {
    std::string reply;

    // Echo is decided by the state as it stood when the command arrived, which
    // is what a real ELM327 does: "ATE0" is itself echoed, and only the command
    // after it is not.
    const bool echo = state_.echo;

    if (isAtCommand(line)) {
        switch (applyAtCommand(line, state_)) {
            case AtResult::Ok:       reply = "OK"; break;
            case AtResult::Unknown:  reply = "?";  break;
            case AtResult::Reset:
            case AtResult::Identify: reply = kVersionBanner; break;
            case AtResult::Voltage:  reply = "12.6V"; break;
            case AtResult::NoData:   reply = "NO DATA"; break;
            case AtResult::DeviceDescription:    reply = kDeviceDescription; break;
            case AtResult::DeviceIdentifier:     reply = state_.identifier; break;
            case AtResult::DescribeProtocol:     reply = describeProtocol(state_); break;
            case AtResult::DescribeProtocolNumber:
                                                 reply = describeProtocolNumber(state_); break;
            case AtResult::BufferDump:           reply = formatBufferDump(state_); break;
        }
    } else {
        reply = runRequest(line);
    }

    if (echo) {
        std::string echoed(line);
        echoed += '\r';
        reply = echoed + reply;
    }

    reply += '\r';
    reply += kPrompt;
    return reply;
}

std::string Elm327Session::runRequest(const char* hexLine) {
    uint8_t payload[8];
    size_t payloadLen = 0;
    if (!hexLineToBytes(hexLine, payload, sizeof(payload), payloadLen)) {
        return formatFault("?");
    }

    twai_message_t request = {};
    request.identifier = state_.header;
    // ATV1 sends only the bytes used; ATV0 (the default) pads to 8, which is
    // what almost every ECU expects.
    request.data_length_code = state_.variableDlc
        ? static_cast<uint8_t>(state_.extendedAddressing ? payloadLen + 2 : payloadLen + 1)
        : 8;
    if (!buildSingleFrameRequest(payload, payloadLen, request.data,
                                 state_.extendedAddressing,
                                 state_.extendedAddress)) {
        return formatFault("?");
    }

    // Drain anything left in the 64-deep RX queue before transmitting. The
    // session is half-duplex and nothing is outstanding at this instant, so
    // whatever is queued is provably stale — a reply that arrived after its own
    // request had already timed out into NO DATA. Consuming it as the answer to
    // THIS request would report one PID's bytes as another's, and the desync
    // would persist for every later poll until reboot.
    {
        twai_message_t stale = {};
        while (canBusReceive(stale, 0)) {}
    }

    if (!canBusTransmit(request, 100)) {
        return formatFault("CAN ERROR");
    }

    // ATR0: fire and forget. The client has said it does not want a reply, so
    // waiting out the full timeout would only stall the next command.
    if (!state_.responses) return "OK";

    IsoTpReassembler reassembler;
    const uint32_t deadline = millis() + state_.timeoutMs;

    while (millis() < deadline) {
        twai_message_t rx = {};
        if (!canBusReceive(rx, 10)) continue;

        // A response to a request sent to 0x7Ex arrives on 0x7Ex + 8.
        // Requests to the functional address 0x7DF are answered by any ECU.
        const bool addressed = (state_.header == 0x7DF)
            ? (rx.identifier >= 0x7E8 && rx.identifier <= 0x7EF)
            : (rx.identifier == static_cast<uint32_t>(state_.header) + 8);
        if (!addressed) continue;

        // Keep the newest accepted frame so ATBD has something to report.
        // This is the RAW frame, address byte and all: ATBD dumps exactly what
        // came off the bus, not what the reassembler made of it.
        state_.lastFrame.valid = true;
        // A malformed frame can carry a DLC of 9-15; formatBufferDump only
        // ever prints 8 bytes, so clamp here rather than let the two disagree.
        state_.lastFrame.dlc = (rx.data_length_code > 8) ? 8 : rx.data_length_code;
        std::memcpy(state_.lastFrame.data, rx.data, 8);

        // Extended addressing puts the target's own address in data[0] on the
        // way out (buildSingleFrameRequest), and the ECU mirrors that same
        // convention on the way back: its reply also leads with an address
        // byte before the ISO-TP PCI byte. The reassembler only understands
        // ISO-TP, so the receive side must undo what the transmit side added
        // — strip that leading byte here, or the reassembler reads the
        // address as a PCI type and every extended-addressing request fails.
        const uint8_t* isoTpFrame = rx.data;
        size_t isoTpLen = rx.data_length_code;
        if (state_.extendedAddressing) {
            if (isoTpLen < 2) continue;  // no room for address byte + PCI
            ++isoTpFrame;
            --isoTpLen;
        }

        switch (reassembler.offer(isoTpFrame, isoTpLen)) {
            case IsoTpState::Complete: {
                const std::vector<uint8_t>& body = reassembler.payload();
                switch (classifyResponse(payload, payloadLen,
                                         body.data(), body.size())) {
                    case ResponseMatch::Positive:
                        return formatPayload(body.data(), body.size(), state_);
                    case ResponseMatch::NegativeFinal:
#if defined(REPORT_NRC)
                        // Scan build only. The DID scanner must tell "this DID
                        // does not exist" (NRC 31) from "this ECU did not
                        // answer" (timeout), and from "this DID exists but is
                        // session-gated" (NRC 7E/7F). Collapsing all three into
                        // NO DATA makes the scan unable to report what it found.
                        // classifyResponse has already established that body is
                        // at least 3 bytes of 7F <service> <nrc>.
                        return formatNrc(body[2]);
#else
                        // A genuine rejection (service not supported, wrong
                        // session, conditions not correct...). Forwarding the
                        // NRC bytes as a payload would have the app decode them
                        // as signal values.
                        return formatFault("NO DATA");
#endif
                    case ResponseMatch::NegativePending:
                        // 7F xx 78: the ECU is still working. A real ELM327
                        // keeps waiting, so keep waiting — bounded by the same
                        // deadline, so this cannot loop forever.
                    case ResponseMatch::Mismatch:
                        // Someone else's answer. Drop it and keep listening.
                        reassembler.reset();
                        break;
                }
                break;
            }
            case IsoTpState::NeedFlowControl: {
                twai_message_t fc = {};
                fc.identifier = state_.header;
                fc.data_length_code = 8;
                buildFlowControlFrame(fc.data);
                if (!canBusTransmit(fc, 50)) return formatFault("CAN ERROR");
                break;
            }
            case IsoTpState::Collecting:
                break;
            case IsoTpState::Error:
            case IsoTpState::Idle:
                return formatFault("CAN ERROR");
        }
    }

    return formatFault("NO DATA");
}
