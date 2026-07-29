# CAN-FD bus test

> **The `rejsacan_bustest` environment has been removed.** The procedure below
> is kept because the question it answers is still open. To run it again,
> restore the env and its `BUS_TEST_ONLY` blocks in `src/main.cpp` from git
> history — `canBusBegin(true)` and `canBusCounters()` are still in `can/`.

Answers one question: does the IONIQ 5's OBD port carry classic CAN that the
ESP32-S3's TWAI controller can decode, or CAN-FD that it cannot?

This gates the SD logging feature. Diagnostic polling works either way — that
is how commercial ELM327s read these PIDs — but full-bus capture needs the
broadcast traffic to be classic CAN.

## Running it

    cd esp32-s3
    pio run -e rejsacan_bustest -t upload
    pio device monitor

Plug the board into the OBD port with the car AWAKE (doors unlocked, ignition
on or actively charging — a sleeping car sends nothing and proves nothing).
The board is listen-only: it cannot transmit, cannot ACK, and cannot disturb
the bus.

Watch for 60 seconds.

## Reading the result

| Observation | Meaning |
|---|---|
| `frames` climbing steadily (hundreds/sec), `busErrors` low and flat | Classic CAN. Full-bus logging is viable — proceed with Task 10. |
| `frames` at or near 0, `busErrors` climbing fast | The controller sees traffic it cannot decode. Consistent with CAN-FD. Full-bus logging is NOT possible on this board. |
| `frames` 0 AND `busErrors` 0 | Nothing on the wire — car asleep, wrong pins, or CAN_RS floating. Not a valid result; retry with the car awake. |
| `rxMissed`/`rxOverrun` climbing | Frames arriving faster than the loop drains them. Only a throughput note, not an FD signal. |

## If it is CAN-FD

The ELM327 half of this project stands on its own and is unaffected. Full-bus
logging would need hardware with an FD-capable controller (e.g. an MCP2518FD).
Record the outcome in the design spec and skip Task 10.
