#include "link/ble_link.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Arduino.h>

namespace {

// ELM327 BLE clones expose this profile; the Android app probes for it.
constexpr char kElmService[] = "0000FFE0-0000-1000-8000-00805F9B34FB";
constexpr char kElmChar[]    = "0000FFE1-0000-1000-8000-00805F9B34FB";

// Nordic UART Service — a generic BLE serial pipe for debugging.
constexpr char kNusService[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char kNusTx[]      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char kNusRx[]      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";

void (*g_handler)(const char*, std::string&) = nullptr;
BLECharacteristic* g_elmChar = nullptr;
BLECharacteristic* g_nusTx = nullptr;
BLEServer* g_server = nullptr;

// ELM327 is stateful and half-duplex: the current ATSH header and echo/space
// settings persist between commands. Two clients issuing commands would
// interleave replies and corrupt both sessions, so the first connection owns
// the session and later ones are dropped.
uint16_t g_ownerHandle = 0;
bool g_hasOwner = false;

std::string g_rxBuffer;

// Longest legitimate ELM327 command (e.g. "ATSH7E0") is well under 20 bytes.
// A client that never sends '\r' — a half-open connection, a client that
// dies mid-command, or a terminal that only sends '\n' — would otherwise
// grow g_rxBuffer without limit, which on an ESP32 means heap exhaustion and
// a crash. Cap it, and on overflow discard rather than truncate: a partial
// command is not recoverable, and keeping a fragment would make the next
// real command parse as garbage.
constexpr size_t kMaxRxBuffer = 256;

// Set synchronously by BLECharacteristicCallbacks::onStatus. For plain
// notifications (as opposed to indications), the library invokes onStatus
// from inside notify() itself before notify() returns — see
// BLECharacteristic::notify() in the installed NimBLE-backed BLE library —
// so checking this flag right after a notify() call reflects that specific
// attempt, not a stale one.
bool g_notifyOk = true;

void onNotifyStatus(BLECharacteristicCallbacks::Status s) {
    g_notifyOk = (s == BLECharacteristicCallbacks::SUCCESS_NOTIFY);
}

void notifyChunk(BLECharacteristic* channel, const std::string& piece) {
    channel->setValue(reinterpret_cast<const uint8_t*>(piece.data()), piece.size());
    g_notifyOk = true;  // optimistic; onNotifyStatus overwrites synchronously on failure
    channel->notify();
    if (!g_notifyOk) {
        // The host's outgoing notification queue was full (ERROR_GATT, e.g.
        // BLE_HS_ENOMEM). This is the only case that needs a wait: give the
        // controller a brief moment to drain and retry once. We deliberately
        // do not retry more than once or loop until success — this callback
        // runs on the NimBLE host task, and a wait here blocks all other BLE
        // host processing (including, notably, the disconnect of a second,
        // rejected client). A longer/looping retry would need to move the
        // reply off the host task entirely (e.g. a queue serviced from
        // loop()), which is a bigger change than this single-connection
        // device's risk justifies.
        delay(2);
        g_notifyOk = true;
        channel->notify();
    }
}

// Largest notification payload the peer will accept: ATT_MTU minus the 3-byte
// notification header. The Android app negotiates MTU 185, which turns a
// 61-byte multi-frame reply from eight notifications into one.
//
// Falls back to the 23-byte default MTU (20 bytes of payload) whenever the real
// value isn't known: ble_att_mtu() returns 0 for an unknown handle, and MTU
// exchange may not have completed yet. Under-estimating only costs extra
// notifications; over-estimating would have the stack silently truncate the
// reply, so the floor is never crossed.
size_t chunkSize() {
    if (!g_server || !g_hasOwner) return 20;
    const uint16_t mtu = g_server->getPeerMTU(g_ownerHandle);
    return mtu > 23 ? static_cast<size_t>(mtu - 3) : 20;
}

void deliver(const std::string& text, bool viaNus) {
    BLECharacteristic* channel = viaNus ? g_nusTx : g_elmChar;
    if (!channel) return;
    // Chunk rather than truncate — the client reassembles by scanning for the
    // '>' prompt, so the split point never matters.
    const size_t chunk = chunkSize();
    for (size_t offset = 0; offset < text.size(); offset += chunk) {
        notifyChunk(channel, text.substr(offset, chunk));
    }
}

void feed(const std::string& incoming, bool viaNus) {
    if (g_rxBuffer.size() + incoming.size() > kMaxRxBuffer) {
        // Unrecoverable partial command — drop it and start clean rather
        // than parsing a stitched-together fragment as if it were real
        // input. Reply with a bare "?" (ELM327's standard error reply) so a
        // well-behaved client waiting on the prompt isn't left hanging.
        g_rxBuffer.clear();
        deliver("?\r>", viaNus);  // matches Elm327Session::handleLine's own "?" + '\r' + prompt shape
        return;
    }
    g_rxBuffer += incoming;
    size_t cr;
    while ((cr = g_rxBuffer.find('\r')) != std::string::npos) {
        const std::string line = g_rxBuffer.substr(0, cr);
        g_rxBuffer.erase(0, cr + 1);
        if (line.empty() || !g_handler) continue;
        std::string reply;
        g_handler(line.c_str(), reply);
        deliver(reply, viaNus);
    }
}

// BLECharacteristic::getValue() returns an Arduino String, not std::string
// (this differs from the brief, which assumed a std::string-returning API).
// feed() takes std::string, so convert at the boundary.
std::string toStdString(const String& s) {
    return std::string(s.c_str(), s.length());
}

class ElmCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override { feed(toStdString(c->getValue()), false); }
    void onStatus(BLECharacteristic*, Status s, uint32_t) override { onNotifyStatus(s); }
};

class NusCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override { feed(toStdString(c->getValue()), true); }
};

// g_nusTx is notify-only (no onWrite to handle), so it gets its own
// status-only callback object rather than sharing NusCallbacks.
class NusTxStatusCallbacks : public BLECharacteristicCallbacks {
    void onStatus(BLECharacteristic*, Status s, uint32_t) override { onNotifyStatus(s); }
};

// The brief's code assumed the Bluedroid two-argument callback form
// (onConnect(BLEServer*, esp_ble_gatts_cb_param_t*)). This core (Arduino-ESP32
// 3.3.9, as pulled in by the pioarduino platform release pinned in
// platformio.ini) builds the BLE library against NimBLE by default
// (CONFIG_BT_NIMBLE_ENABLED=1 in sdkconfig.h), not Bluedroid, so that overload
// doesn't exist and esp_ble_gatts_cb_param_t isn't even declared. The NimBLE
// two-argument form is onConnect(BLEServer*, ble_gap_conn_desc*) and carries
// the connection handle in desc->conn_handle instead of
// param->connect.conn_id. Switched to that form to keep the connection-id
// based arbitration the brief intended.
class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* server, ble_gap_conn_desc* desc) override {
        if (g_hasOwner) {
            // Someone already owns the session. Drop the newcomer rather than
            // letting two clients interleave commands.
            server->disconnect(desc->conn_handle);
            return;
        }
        g_hasOwner = true;
        g_ownerHandle = desc->conn_handle;
        g_rxBuffer.clear();
    }

    void onDisconnect(BLEServer* server, ble_gap_conn_desc* desc) override {
        // Only the owner's departure ends the session.
        //
        // This guard is load-bearing, not defensive. Rejecting a second client
        // works by disconnecting it, which fires this same callback for the
        // NEWCOMER's handle. Releasing ownership here unconditionally would
        // mean the arbitration is defeated by the very mechanism meant to
        // enforce it: the owner stays connected but unowned, a third client can
        // then seize the session, and the owner's half-typed command is wiped
        // mid-stream. The bench test would still show the newcomer being
        // dropped and look correct.
        if (!g_hasOwner || desc->conn_handle != g_ownerHandle) {
            // A rejected client going away. Keep advertising so the next one
            // can still find the board, and leave the owner untouched.
            server->startAdvertising();
            return;
        }

        g_hasOwner = false;
        g_rxBuffer.clear();
        server->startAdvertising();
    }
};

} // namespace

void bleLinkSetLineHandler(void (*handler)(const char*, std::string&)) {
    g_handler = handler;
}

void bleLinkBegin(const char* deviceName) {
    BLEDevice::init(deviceName);
    g_server = BLEDevice::createServer();
    g_server->setCallbacks(new ServerCallbacks());

    BLEService* elm = g_server->createService(kElmService);
    g_elmChar = elm->createCharacteristic(
        kElmChar,
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_WRITE_NR |
        BLECharacteristic::PROPERTY_NOTIFY);
    g_elmChar->setCallbacks(new ElmCallbacks());
    elm->start();

    BLEService* nus = g_server->createService(kNusService);
    g_nusTx = nus->createCharacteristic(kNusTx, BLECharacteristic::PROPERTY_NOTIFY);
    g_nusTx->setCallbacks(new NusTxStatusCallbacks());
    BLECharacteristic* nusRx = nus->createCharacteristic(
        kNusRx,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    nusRx->setCallbacks(new NusCallbacks());
    nus->start();

    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(kElmService);
    advertising->addServiceUUID(kNusService);
    advertising->setScanResponse(true);
    BLEDevice::startAdvertising();
}
