# RejsaElm — an ELM327 for the RejsaCAN

Firmware turning a **RejsaCAN-ESP32-S3** into an ELM327-compatible OBD-II
adapter over Bluetooth LE.

It advertises as `RejsaElm` and speaks the ELM327 AT dialect, so it works with
any BLE-capable OBD tool. Nothing about it is IONIQ 5-specific — it is a
generic adapter that happens to have been built for one.

## Build and flash

```sh
pio test -e native            # 96 host tests, no hardware needed
pio run -e rejsacan -t upload # build and flash the board
pio device monitor            # 115200 baud
```

The Python tools under `tools/` have their own environment, managed by
[uv](https://docs.astral.sh/uv/):

```sh
uv sync                       # create .venv from pyproject.toml
uv run pytest                 # 156 tests for did_scan.py and didscan_core.py
```

Three environments:

| Environment | Purpose |
|---|---|
| `rejsacan` | The adapter firmware |
| `rejsacan_usbscan` | Scan-only build for `tools/did_scan.py`; see "DID scanning" below |
| `native` | Host-side unit tests for the pure-logic units |

## Architecture

```
main.cpp    wiring and task startup

link/       BLE / USB CDC / WiFi TCP, single-session arbitration    hardware
can/        TWAI: init, transmit, receive                        hardware
elm327/     session: request/response over CAN                   hardware

elm327/     AT command parser, response formatter                PURE LOGIC
elm327/     hex line parsing, response correlation               PURE LOGIC
isotp/      segmentation and reassembly                          PURE LOGIC
```

The pure-logic units carry no hardware dependency and run under PlatformIO's
`native` environment on the development machine. That is deliberate: on the
Android side of this project, every substantive bug found — a dropped nibble, a
timeout desync, a missing response type — was caught in pure-logic tests rather
than on hardware. Debugging ISO-TP framing by reflashing a board in a parked car
is the loop this structure exists to avoid.

## Bluetooth

Two GATT services carry the same byte stream:

| Service | Characteristics | Purpose |
|---|---|---|
| `FFE0` | `FFE1` (write + notify) | What ELM327 clones expose, and what the Android app probes for. The primary path. |
| Nordic UART `6E400001-…` | `6E400003` notify, `6E400002` write | A generic serial pipe, so nRF Connect and similar can reach the board for debugging |

**One session at a time.** ELM327 is stateful — the current `ATSH` header and
the echo/space settings persist between commands — so two concurrent clients
would interleave replies and corrupt both. The first to connect owns the
session; a second connection is dropped immediately.

## WiFi

WiFi is off by default and opt-in at build time with `-DWIFI`, which adds a TCP
listener to the normal firmware — BLE and WiFi then both serve the same
`Elm327Session`. The board runs its own WPA2 access point and listens on **port 35000** — the
convention every ELM327-over-WiFi clone uses, so SavvyCAN, OBD Fusion and
`nc 192.168.4.1 35000` all work with no configuration.

PlatformIO has no `--build-flag`, so the flag goes through the environment:

```sh
PLATFORMIO_BUILD_FLAGS='-DWIFI -DWIFI_PASSWORD=\"hunter2xyz\"' \
  pio run -e rejsacan -t upload
# join SSID "RejsaElm", then:
nc 192.168.4.1 35000
```

**Set `WIFI_PASSWORD`.** It is the only thing between anyone in radio range and
the CAN bus of a parked car. Building without it uses a default published in
this repo and warns at compile time; anything under WPA2's 8-character minimum
fails the build rather than silently failing to bring the AP up.

Station mode (joining an existing network) is not offered: the car is not where
the house WiFi is, and the credentials would need somewhere to live.

**One client across both radios.** `Elm327Session` is half-duplex and stateful,
and BLE callbacks run on the NimBLE host task while `wifiLinkPoll()` runs in
`loop()` — so both links may be *up*, but only the client holding the session is
ever fed to it. Ownership is a single atomic token (`link/session_owner.h`,
tested under `native`); whoever connects first keeps it until they disconnect,
and anyone else — same radio or the other one — is dropped at connect. USB CDC
stays separated at build time: it has no connect/disconnect events to hang a
claim on.

Not on by default: the WiFi stack costs ~415 kB of flash, ~13 kB of RAM and
idle current draw. Without `-DWIFI` the linker drops all of it — verified with
`nm` on the two images, the default build contains no `esp_wifi_*` symbols.

## DID scanning

`env:rejsacan_usbscan` is a scan-only build: the same `Elm327Session`, served
over USB CDC serial instead of BLE, with negative-response codes reported as
`NRC xx` rather than collapsed into `NO DATA`.

    pio run -e rejsacan_usbscan -t upload
    cd tools && python3 did_scan.py --port /dev/tty.usbmodem101 --out scan.jsonl
    python3 did_scan.py --summary scan.jsonl

It exists to inventory which DIDs this car's ECUs actually implement, since the
decode layer is seeded from community lists spanning several model years. See
`docs/superpowers/specs/2026-07-27-did-scan-design.md`.

Two things about this build are deliberate:

- **BLE is not started.** `Elm327Session` is strictly half-duplex and holds
  mutable state; driving it from both the BLE task and `loop()` would race. The
  links are separated at build time rather than synchronised at runtime.
- **NRC reporting is guarded by `-DREPORT_NRC`.** The shipping build must keep
  answering `NO DATA` to a negative response. Be precise about why, because the
  obvious fear is the wrong one: a leaked `NRC 31` would *not* be decoded as a
  pack voltage. `ResponseParser.kt` strips the space to `NRC31`, which fails both
  its even-length and its hex-digit checks, so it returns `Fault(GARBAGE)` — no
  fabricated reading is reachable from that string. (Forwarding the raw `7F 22
  31` *bytes* as a payload would be that hazard, which is what `session.cpp`'s
  own comment is about.) What breaks instead is meaning: every negative response
  becomes `GARBAGE` — documented in `AdapterError.kt` as "unparseable, do not
  guess" — rather than `NO_DATA`, documented as "ECU did not answer. Normal for
  an unsupported PID or a sleeping car". `Poller` currently treats every `Fault`
  identically, so today's polling and backoff would not visibly change; what is
  lost is the ability to tell a rejected DID from adapter noise, in logs and in
  any future code that keys on `NO_DATA`. A real regression, and a diagnostic
  one. `formatter.h` makes the bad build combination a compile error, and
  `test_formatter` pins for every byte value that `formatNrc` output is neither
  a fault word nor parseable as hex.

The Android app cannot talk to this build. Re-flash `env:rejsacan` afterwards.

## Things that are load-bearing

**`CAN_RS` must be driven LOW.** Left floating, the transceiver selects
slope-control mode and silently mangles 500 kbit/s frames. This is the
difference between a working adapter and a mysteriously silent one.

**The wire format is a shared contract.** `formatter.cpp` emits the headers-off
ELM327 rendering — a bare three-hex-digit length line followed by `0:`/`1:`
index-prefixed continuation lines — because that is exactly what the Android
app's `ResponseParser` accepts. Its tests are built from that parser's own
fixtures, so both halves are checked against one definition rather than two
private ones.

**Responses are correlated, not assumed.** A positive UDS response echoes the
request's service byte plus `0x40` (`22` → `62`) and, for service `22`, the
two-byte DID. Both are verified before a payload is returned, and the RX queue
is drained before each transmit. Without this, a reply arriving after its
deadline is handed to the *next* request and decodes as a plausible wrong value.

**Forwarding is unrestricted, deliberately.** The firmware passes on whatever
service a client asks for, including write and control services. A real ELM327
is transparent plumbing and the goal was to be a faithful one; the board is then
no more dangerous than the commercial adapter it replaces. The consequence,
stated plainly: a third-party app connected to this board can issue actuator
tests or ECU writes. That risk is inherent to any ELM327.

## What this board cannot do

**Passive full-bus logging is impossible at the OBD port** — not because of this
hardware, but because of the car. A listen-only capture recorded **zero frames
and zero errors** on an awake vehicle. Zero *errors* is the tell: a CAN-FD bus
would produce climbing error counts, not silence.

Hyundai/Kia place a central gateway between the OBD connector and the internal
buses. It forwards diagnostic request/response traffic and does not bridge
broadcast frames. CSS Electronics document the same on the Kia EV6: the car
"does not provide any 'default broadcast data' via the OBD2 connector".

An SD-logging feature was planned and **cancelled** for this reason. The SD pins
remain in `board_pins.h` as a board map; no logging code exists. Reaching real
broadcast traffic would require physically tapping a bus behind the gateway.

## Hardware

**RejsaCAN-ESP32-S3 v3.x** — ESP32-S3-WROOM-1-N16R8, 16 MB flash, 8 MB OPI
PSRAM, onboard CAN transceiver, powered from the OBD port.

| Pin | Use |
|---|---|
| GPIO 14 / 13 | CAN TX / RX |
| GPIO 38 | `CAN_RS` — drive LOW for high-speed normal mode |
| GPIO 17 | `FORCE_ON`, holds off the auto-shutdown circuit |
| GPIO 11 / 10 | Warning (yellow) and activity (blue) LEDs |
| GPIO 39/40/41/45 | SD SCK/MOSI/MISO/CS — unused, see above |

The ESP32-S3 has **no Bluetooth Classic**, so this cannot serve apps expecting
an SPP serial adapter. Its TWAI controller is **classic CAN only** — no CAN-FD.

## References

| Project | How it helped |
|---|---|
| [MagnusThome/RejsaCAN-ESP32](https://github.com/MagnusThome/RejsaCAN-ESP32) | The board itself: schematics, pin definitions, BOM. Ships no firmware — universal hardware driven by Arduino libraries |
| [meatpiHQ/wican-fw](https://github.com/meatpiHQ/wican-fw) | Open-source ESP32 firmware implementing ELM327/ELM329/STN command sets. WiCAN Pro is ESP32-S3, same silicon — the closest existing prior art |
| [pioarduino platform fork](https://github.com/pioarduino/platform-espressif32) | Arduino core 3.x for ESP32-S3. The official espressif32 platform is frozen at core 2.0.17 |
| ELM327 datasheet (elmelectronics.com) | The specification. There is no standards body — the datasheet *is* the spec for AT semantics, response strings and timing |
| ISO 15765-2 (ISO-TP) | Single/first/consecutive frames, flow control, sequence wrapping |
| [EQMOD/REJSACAN_OBDWEB](https://github.com/EQMOD/REJSACAN_OBDWEB) | A RejsaCAN-ESP32-S3 OBD dongle with an internal webserver — evidence of what this board can do |
| [Unity](http://www.throwtheswitch.org/unity) | The test framework behind the `native` environment |

## Known gaps

- **Not verified on a vehicle.** The bus test has been run; the ELM327 path has
  not been exercised end to end against a car. The DID scanner is the same
  story — every bench and in-car row for it in `tools/bringup_checklist.md` is
  unverified, since the board has not been attached at any point during its
  implementation.
- `initialise()` issues its AT commands and checks each reply, but `ATZ`'s
  version banner is accepted as GARBAGE by design — see the comment there.
- `millis()` overflow in the request deadline yields one spurious `NO DATA` per
  ~49.7 days of uptime. Self-correcting, unfixed.
- A notify that fails twice drops that chunk and continues; the reply is gapped
  rather than lost entirely, and the client still receives its prompt.
- `ATRV` returns a fixed `12.6V`. The board has a real divider on GPIO 9;
  nothing consumes the reading.
