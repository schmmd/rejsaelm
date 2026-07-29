# Full ELM327 command set

Design, 2026-07-28.

Today `at_parser.cpp` implements 28 AT commands. The datasheet has roughly 120.
This closes the gap as far as the hardware allows, so that a generic OBD app —
not just this project's Android client — completes its handshake and gets
truthful answers.

**The target is a general-purpose adapter.** The IONIQ 5 is the vehicle on hand,
not the design centre. Where a decision could be made either for that car or for
any car, this document takes the general option, and a limitation observed on
that one vehicle is not treated as a limitation of the board. The README already
states this intent for the firmware as a whole; it applies here too.

## What "full" can mean on this board

RejsaCAN has a CAN transceiver and nothing else. Protocols 1–5 (J1850 PWM,
J1850 VPW, ISO 9141-2, ISO 14230 5-baud init and fast init) have no electrical
path, and the commands that exist to serve them go with them. `ATBRD`/`ATBRT`
set a UART divisor that means nothing over BLE or USB CDC. CAN-FD is out — the
TWAI controller is classic-only.

Every command therefore lands in one of three piles:

1. **Implemented** — CAN-reachable and observable.
2. **Accepted and ignored** — `OK`, state stored, nothing consults it. The
   codebase already documents this pattern for `ATH1`/`ATL1` in `at_parser.h`.
3. **`?`** — honest refusal.

Pile 3 is a feature, not a shortfall. A client that asks for ISO 9141 init and
is told `OK` will wait for a bus that will never answer.

## Out of scope

**Real `ATH1` header output.** The reassembler discards frame boundaries, CAN
IDs and PCI bytes, so per-frame rendering would mean retaining raw frames
through `runRequest()` and adding a second output format. `ATH1` and `ATCAF0`
stay stored-and-ignored exactly as `at_parser.h` describes them today.

The consequence: `formatPayload()` is not touched by this work. Its signature,
its output, and the `test_formatter` fixtures derived from `ResponseParser.kt`
are all unchanged. If a formatter test moves during this work, that is a bug.

**`ATPP` programmable parameters, `ATSD`/`ATRD`.** 32 persisted parameters with
their own command grammar, and a reputation for bricking clones when set wrong.
Nothing on either side of this project reads them.

Monitor modes and J1939 were both cut in an earlier draft and are now in scope,
as phase 4. The goal is a generally useful adapter, not one shaped around a
single car — and the gateway that makes monitoring useless *here* is a property
of this vehicle, not of the board.

## Phase 1 — pure logic

Confined to `at_parser.cpp`, `at_parser.h` and `AdapterState`. Everything here
runs under `env:native`, which is the point: no hardware needed to prove it.

| Command | Behaviour |
|---|---|
| `ATDP` | `ISO 15765-4 (CAN 11/500)`, from the current protocol. Prefixed `AUTO, ` when selected by `ATSP0` |
| `ATDPN` | `6`, or `A6` when auto-selected |
| `ATR0` / `ATR1` | Responses off/on. `R0` returns as soon as the frame is sent, without waiting for a reply |
| `ATV0` / `ATV1` | Variable DLC. `V1` sets the frame DLC to the bytes actually used instead of the hardcoded 8 |
| `ATCEA hh` / `ATCEA` | CAN extended addressing on/off. Prepends the address byte to each frame; usable payload drops to 6 bytes |
| `ATBD` | Buffer dump — the last raw received frame as hex. Requires retaining one frame, not the whole sequence |
| `ATTA hh` | Tester address |
| `ATCP hh` | Priority bits of a 29-bit ID. Stored; takes effect in phase 3 |
| `ATPC` | Protocol close |
| `@1` | Device description |
| `@2` | Device identifier, empty until set |
| `@3 cccccccccccc` | Set the identifier. RAM only for now; moves to NVS in phase 2 |
| `ATAL` / `ATNL` | Allow/disallow long messages. **Stored, not honoured** — `runRequest()` only builds single frames, so multi-frame transmit does not exist to enable |
| `ATMA`, `ATMR hh`, `ATMT hh` | Interim: immediate `NO DATA`, so a client is not left waiting out a timeout. Becomes real in phase 4 |
| `ATIB`, `ATFI`, `ATSI`, `ATKW`, `ATBI`, `ATSW`, `ATWM`, `ATSP1`–`ATSP5` | `?` |

`@1` currently maps to `AtResult::Identify` and returns the `ATI` version
banner. That is wrong — `@1` is a device description, distinct from `ATI` — and
this phase separates them.

`AdapterState` gains `protocol`, `autoSelected`, `responses`, `variableDlc`,
`extendedAddress`, `testerAddress`, `priorityBits`, `identifier`.

**Tests:** extend `test/test_at_parser`. One case per command, and one that
asserts every pile-3 command returns `Unknown` — that assertion is what stops a
later refactor from quietly making a refusal into an `OK`.

## Phase 2 — hardware-backed

| Command | Behaviour |
|---|---|
| `ATRV` | Real reading from the GPIO 9 divider, formatted `12.6V` |
| `ATCV dddd` | Calibration. `ATCV 1234` asserts the true voltage is 12.34 V; the scale factor is derived and persisted in NVS |
| `ATCS` | TX and RX error counters from `twai_get_status_info()` |
| `ATLP` | Deep sleep, waking on timer or CAN activity |

`ATRV` needs the calibration knob, not just the divider ratio from the
schematic. Real resistors are not nominal and the ADC has its own offset; a
formula with no adjustment will be wrong by a tenth of a volt and there will be
no way to correct it.

`ATIGN` is an open question. GPIO 17 is `FORCE_ON`, an *output* that holds off
the auto-shutdown circuit — it is not an ignition sense input, and the board has
no other. The board is powered from the OBD port, so if it is running at all the
answer is arguably `ON`. Decide when implementing: constant `ON` with a comment
explaining the absence of a real input, or `?`.

## Phase 3 — protocol and filtering

The largest phase, and the only one that touches `session.cpp` structurally.

- **`ATSP` 6/7/8/9, `ATSPA n`, `ATTP n`.** Protocols 7 and 9 are 29-bit, which
  changes the response-ID correlation in `runRequest()` — the `header + 8` rule
  and the `0x7DF` functional-broadcast special case are both 11-bit assumptions.
  Protocols 8 and 9 are 250 kbit/s, which means tearing down and reinitialising
  TWAI at a new bit rate.
- **`ATSP0` auto-search** across the above, setting `autoSelected` so `ATDPN`
  reports `A6`.
- **`ATCRA`, `ATCF`, `ATCM`** for real. Accepted and ignored today. A software
  filter in the receive loop is sufficient; the TWAI hardware acceptance filter
  is not worth configuring for this.
- **`ATFC SH` / `SD` / `SM`** for real. `buildFlowControlFrame()` currently
  hardcodes its frame.
- **`ATAT 0/1/2`** adaptive timing, replacing the fixed `state_.timeoutMs`
  deadline.

## Phase 4 — streaming, monitor modes, J1939

Depends on phase 3: J1939 is ELM327 protocol A, which is CAN at 250 kbit/s with
29-bit IDs, so it reuses the bit-rate switching and extended addressing built
there rather than introducing them.

### 4a — streaming

The prerequisite for everything else in this phase, and the only structural
change to the session.

`handleLine()` is strictly request → reply → prompt. Monitoring is
reply-until-cancelled: frames stream out as they arrive, and any byte received
from the client aborts and returns the prompt. That means a monitor mode cannot
be a return value from `handleLine()` — it needs a session state the link layer
pumps, plus a cancellation path from the link back into the session.

Once that exists, `ATMA`, `ATMR hh` and `ATMT hh` become real, replacing their
phase 1 stubs.

On the IONIQ 5 these will emit nothing, because its gateway does not bridge
broadcast traffic to the OBD port. That is a fact about that car. On a vehicle
without a consumer gateway — including any J1939 machine — the same code is the
primary way the bus is read.

### 4b — J1939 addressing

A 29-bit J1939 identifier decomposes into 3 bits priority, 1 reserved, 1 data
page, 8 bits PDU Format, 8 bits PDU Specific, 8 bits source address. The PGN
derivation is conditional and is the part implementations most often get wrong:

- **PF < 240 (PDU1)** — destination-specific. PS *is* the destination address,
  and the PGN is `PF << 8`.
- **PF ≥ 240 (PDU2)** — broadcast. PS is a group extension, and the PGN is
  `(PF << 8) | PS`.

This is pure logic and belongs under `env:native` with the other decoders.

| Command | Behaviour |
|---|---|
| `ATJE` | ELM data format — request bytes in the order given. Default |
| `ATJS` | SAE data format — reverses PGN byte order to match how SAE prints it |
| `ATJHF0` / `ATJHF1` | Header formatting off/on: raw header, or decoded into priority / PGN / source address |
| `ATJTM1` / `ATJTM5` | Timer multiplier 1× or 5×, for the slower J1939 transport transfers |
| `ATMP hhhh` / `ATMP hhhh n` | Monitor for a PGN. Needs 4a |
| `ATDM1` | Monitor for DM1, the broadcast active-fault-code message. A PGN 65226 filter over `ATMP`; no DTC decoding — frames are emitted in the normal format |

### 4c — J1939 transport protocol

Messages over 8 bytes use J1939's own transport, not ISO-TP. Two modes:

- **BAM** — `TP.CM_BAM` (PGN 60416) announces size and frame count, then
  `TP.DT` (PGN 60160) frames follow with no flow control.
- **RTS/CTS** — destination-specific, with CTS frames pacing the sender.

This is a second reassembler beside `IsoTpReassembler`, and deliberately the
same shape: offer frames, collect, report complete. It carries no hardware
dependency, so it is host-testable exactly like the existing one — which is the
strongest argument that this phase fits the codebase rather than fighting it.

### Verifying it

Phase 4 cannot be tested on the IONIQ 5 at all. The reassembler and the
addressing decode are host-testable and should carry the weight. Beyond that,
verification needs either a J1939 bench source — a second CAN node replaying a
logged capture — or access to a truck. Note this in
`tools/bringup_checklist.md` rather than letting the phase look verifiable when
it is not.

## Risks

**Protocol switching is the one that can break a working adapter.** Phases 1 and
2 are additive — no existing path changes behaviour. Phase 3 rewrites the
correlation logic that `session.cpp` documents as load-bearing: "a reply
arriving after its deadline is handed to the *next* request and decodes as a
plausible wrong value." The 11-bit path must keep working byte-for-byte while
29-bit support is added beside it, which argues for pinning the existing
correlation behaviour in host tests before touching it.

**Streaming is the other one.** Phase 4a puts the session into a state where it
emits without being asked, and the link layer must be able to interrupt it. The
BLE link already drops a second connection to keep one session authoritative;
a monitor mode that ignores its cancellation path would hold that session open
indefinitely, and the only recovery would be a power cycle in a parked car.
Cancellation is the load-bearing part of 4a, not the streaming.

**Nothing here is verified on a vehicle**, consistent with the rest of the
project's known gaps. Phase 1 is fully covered by host tests, as are the J1939
addressing decode and transport reassembler in phase 4. Phases 2, 3 and the
monitor modes are not, and their entries belong in
`tools/bringup_checklist.md`. Phase 4 additionally cannot be exercised on the
vehicle on hand at all — see above.
