# ELM327 bring-up checklist

Run after `pio run -e rejsacan -t upload`. Bench first, car second.

## Bench (no vehicle)

| Check | How | Expected |
|---|---|---|
| Board advertises | Open nRF Connect app, rescan devices | A device named `RejsaElm` appears in the list |
| Services are present | Tap to connect to `RejsaElm` in nRF Connect | Two services appear: `FFE0` and `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` |
| NUS pipe works | In nRF Connect, subscribe to `6E400003` (TX), write `ATI\r` to `6E400002` (RX) | See `ELM327 v1.5` in the log, followed by `>` |
| AT echo | Write `ATE1\r` to RX | See `OK` then `>` |
| AT-setting persistence | Write `ATS0\r` to RX | See `OK` then `>` |
| Unknown command rejected | Write `ATXYZZY\r` to RX | See `?` then `>` (unrecognized AT command) |
| Bus failure clear | Write `0100\r` to RX (OBD query with no vehicle CAN attached) | See `NO DATA` then `>` (no answer from the bus) |
| Second client refused | In nRF Connect, keep first connection live; open another device scan and tap `RejsaElm` again from a second BLE app (or simulator) | Second connection attempt fails immediately; first connection stays alive and keeps working |
| `ATDP` reports the protocol | Write `ATDP\r` to RX | See a human-readable protocol description (`AUTO, ISO 15765-4 (CAN 11/500)` or similar) then `>`. **Not host-tested against session.cpp** — `describeProtocol()` is unit-tested, but this row confirms `handleLine()` actually wires it through. |
| `ATDPN` reports the protocol number | Write `ATDPN\r` to RX | See the short numeric form (e.g. `6` or `A6` if auto-selected) then `>`. **Not host-tested against session.cpp**, same caveat as `ATDP` above. |
| `@1` reports the device description | Write `@1\r` to RX | See `RejsaElm OBD-II Adapter` (not the `ELM327 v1.5` version banner) then `>`. **Not host-tested against session.cpp.** |
| `@2`/`@3` round-trip the device identifier | Write `@3RejsaElmBd1\r` then `@2\r` to RX | `@3...` returns `OK`; `@2` echoes back the 12 characters set by `@3`, then `>`. **Not host-tested against session.cpp** (the state round-trip is unit-tested, but not through `handleLine()`). |
| `ATBD` reports a real frame | With a vehicle attached and awake, write `0100\r` then `ATBD\r` to RX | `ATBD` returns the DLC and data bytes of the frame the ECU actually sent back, matching what was seen in the `0100` response. **Never run against a real frame** — `formatBufferDump()` is unit-tested against synthetic `ReceivedFrame` values, but nothing has confirmed `runRequest()` actually populates `state_.lastFrame` from bus traffic. |
| `ATR0` fire-and-forget | Write `ATR0\r` then `0100\r` to RX | `0100` returns `OK` immediately (no wait for a reply), rather than the payload or `NO DATA` after a timeout. **Never run** — confirm the request is still transmitted on the bus (e.g. with a CAN sniffer) even though the adapter does not wait for or report the answer. |
| `ATV1` variable DLC | Write `ATV1\r` then `0100\r` to RX, sniffing the CAN bus | The transmitted frame's DLC is `2` (service + PID, no padding) rather than `8`. **Never run** — confirm with a bus analyzer, since the adapter's own reply gives no direct evidence of the frame's DLC. |
| `ATCEA` against an extended-addressing ECU | Write `ATCEAxx\r` (target extended address) then a request to RX, against a real ECU known to use extended addressing | The ECU responds normally, proving the address byte was placed correctly in the frame and the usable payload was shortened as expected. **Never run** — no extended-addressing ECU has been available during this work; regular (non-extended) addressing is all that has been exercised. |

### DID scanner (`env:rejsacan_usbscan`, `tools/did_scan.py`)

**None of the rows below have been run against real hardware.** The board has
not been attached at any point during this tool's implementation — every row
here is a procedure to follow, not a record of something that passed. Two bugs
were found and fixed purely by re-reading the code and tests during
implementation (an `ATE0` echo mismatch the adapter's reply parsing didn't
account for, and a false-positive "car asleep" abort in the sweep's
silence detector) — both are exactly the kind of thing that would have shown
up the moment this tool first touched a real board. Treat these rows as the
real first exercise of this code against hardware, not a formality.

| Check | How | Expected |
|---|---|---|
| Scan build serves USB | `pio run -e rejsacan_usbscan -t upload`, then `pio device monitor`, type `ATI` | `ELM327 v1.5` then `>` — over USB, with no BLE advertising |
| Scan build times out cleanly with no vehicle | Still in the monitor, type `220101` with no vehicle attached | `NO DATA` then `>`. This shows the scan build still reports a plain timeout the normal way when nothing is on the bus — it does **not** exercise NRC reporting, since there is no ECU present to send a negative response. To see an actual `NRC xx` line, service `22` must reach a real ECU that rejects the request. |
| Scanner initialises | `cd tools && python3 did_scan.py --port <device>` | `adapter ready; 1536 DIDs per ECU, ATST=50 ms` (confirmed against the shipped default ranges: `0100-02FF` + `B000-B0FF` + `C000-C0FF` + `E000-E0FF` + `F100-F1FF` = 512 + 4×256 = 1536) |
| Scanner detects the wrong build | Flash `pio run -e rejsacan -t upload` (the shipping BLE build, which never wires `Serial` to an ELM327 parser — see `main.cpp`'s `loop()`), re-run the scanner | Exits 1 with `init failed: timed out waiting for the prompt after 'ATZ'; got b''` — a bare I/O timeout, since nothing answers on that port at all. (The banner-mismatch message that names `env:rejsacan_usbscan` explicitly only fires if *something* answers `ATZ` without the right banner; the shipping build doesn't answer at all.) |
| Scanner survives no car | Re-flash the scan build, `python3 did_scan.py --port <device>` | `discovering ECUs on 700-7E7 (231 addresses, 7DF excluded)...`, then `no ECUs responded. Is the car awake and in Ready mode?`, exit 1. **Expect this to take around four minutes, not seconds**, and do not mistake it for a hang: discovery probes at `--read-timeout` (500 ms), because a VIN reply is multi-frame and one truncated at the fast sweep deadline would drop a whole ECU from the run. 231 addresses x 2 identification DIDs x 500 ms is the floor with nothing on the bus. |
| Scan log is resumable | `python3 did_scan.py --port <device> --ecu 7E4 --range 0100-0110 --out bench.jsonl`, then re-run with `--resume` | Second run reports `resuming: 17 pairs already recorded` and probes nothing |

## In the car

**Prerequisites:**
- The RejsaCAN board must be plugged firmly into the vehicle's OBD-II port.
- The car must be **awake** — turn on the ignition (do not start the engine). A sleeping car will not respond to any diagnostic request.

| Check | How | Expected |
|---|---|---|
| App finds the board | Open the Android app, look at the device list | `RejsaElm` appears and is marked as available to connect |
| App connects | Tap `RejsaElm` in the app | App shows "connected" status and enters the initialisation phase |
| Initialisation succeeds | Watch the app startup screen | No error dialog about failed initialisation; app proceeds to live data dashboard |
| Live data appears | Observe the app dashboard for 30 seconds | SoC, pack voltage (in V), and cell temperatures start to populate and refresh every few seconds |
| Pack voltage is sane | Check the value shown under "Pack Voltage" in the app | Value is between 500 V and 800 V (typical for an 800 V vehicle) |
| Cell voltage is sane | Check the cell voltage range shown in the app | Minimum cell is above 3.0 V, maximum below 4.2 V |
| SoC is close to dash | Compare app SoC to the vehicle's dash battery meter | Values differ by no more than 5 percentage points |
| Backoff when car sleeps | Turn off the vehicle, leave the board plugged in, watch the app for 3 minutes | App stops requesting data; any "reconnecting" spinners stop within 180 seconds |

### DID scanner (`env:rejsacan_usbscan`, `tools/did_scan.py`)

**Unverified, aspirational.** These two rows are the scanner's actual acceptance
test, and neither has been run — the board has never been in the car for this
work. Re-flash `env:rejsacan_usbscan` first; the Android app cannot talk to
that build, so flash `env:rejsacan` back afterwards.

| Check | How | Expected |
|---|---|---|
| Scan finds the known ECUs | `cd tools && python3 did_scan.py --port <device> --out scan.jsonl` with the car in Ready mode | Discovery reports at least `7E4`, `7E5`, `7B3`, `7A0`, `7C6`. The first implementation could **never** pass this row: it built its candidate list as `range(0x700, 0x7DF)`, and `0x7DF` sits below `0x7E0`, so the whole OBD-II physical block `0x7E0`–`0x7E7` — including the BMS at `7E4` and the ICCU at `7E5` — was silently never probed. If any of the five is missing, check the "discovering ECUs on ..." line names a range that contains it before looking anywhere else. |
| Scan confirms the polled PIDs | `python3 did_scan.py --summary scan.jsonl` | `220101`, `220105` on `7E4`; `22E011` on `7E5`; `220100` on `7B3`; `22C00B` on `7A0`; `22B002` on `7C6` — the exact six requests `android/app/.../PollTier.kt` polls today (the signal definitions themselves now come from the vendored OBDb set, not a hand-written table). **Read this row against the previous one first.** A missing PID is a finding about the app's decode tables *only if discovery found that PID's ECU at all*: if the ECU never appeared, this tells you nothing about the decode tables, and the scanner (or the car being asleep, or the connector) is what to investigate. Any row flagged `unstable` (payload changed between the sweep and the Phase 4b re-read) or `unconfirmed` (the re-read never came back `POSITIVE`) should be treated with suspicion rather than trusted at face value. |

## Troubleshooting

**If the app device list is empty or `RejsaElm` never appears:**
1. Check that the board has power (LED should be lit).
2. On the board, press BOOT and then RST to manually reset.
3. In the app, force-kill and reopen it; rescan the Bluetooth list.
4. **If still not found:** The board may not have programmed correctly. Reconnect to the vehicle's OBD port to rescan for CAN activity, then re-run `pio run -e rejsacan -t upload` and monitor the serial log with `pio device monitor` for boot messages.

**If the app connects but the dashboard shows "—" for all values:**
1. **Check the car is awake.** Turn on the ignition (do not start the engine). The car's CAN network will wake. Wait 10 seconds.
2. **Check CAN connection.** Verify that the RejsaCAN board is firmly seated in the OBD port. The white LED should turn on when CAN activity is detected.
3. **If you see "NO DATA" repeatedly in the serial monitor:** Confirm the car is awake (ignition on), that the OBD connector is fully seated and making solid contact, and that this vehicle supports the diagnostic requests the app is sending. The firmware runs at a fixed 500 kbit/s (correct for Hyundai/Kia vehicles); this is not configurable and cannot be wrong if the hardware connection is good. Try wiggling the OBD connector or resetting the board, then try again.
4. **If the serial monitor shows no errors but the app still has no data:** The car may not support the OBD-II pids (diagnostic requests) that the app is polling. This is not a firmware issue — it is a limitation of the vehicle's diagnostic gateway. Contact the team with the vehicle's year and model.

**If values appear in the app but look wrong (e.g., pack voltage 1000 V, negative SoC, etc.):**
This indicates a signal-table mismatch in the Android app. The firmware is transmitting data correctly (if you see garbled text in the serial monitor, escalate), but the app's byte offsets for cell voltages or pack voltage are incorrect for this car. Do not troubleshoot firmware; this is an Android app verification problem.

**If you see garbled characters in the serial monitor (not readable text) or repeated "CAN ERROR" messages:**
1. Verify the OBD port connection is clean and firm.
2. Reset the board: press BOOT + RST.
3. Re-upload firmware: `cd esp32-s3 && pio run -e rejsacan -t upload`.
4. If garbling persists: There may be a hardware fault (short circuit on CAN lines, bad connector). Do not continue — the board or OBD port may be damaged.

---

**Notes for troubleshooters:**
- The CAN bitrate is fixed at 500 kbit/s — it is correct for this vehicle and cannot be changed.
- The vehicle only forwards diagnostic request/response traffic through the OBD port; there is no broadcast CAN data. This is why a bench test with no vehicle attached returns `NO DATA` — that is correct and expected.
- If the Android app crashes or freezes: Check the app's logs (Android Studio, or exported bug reports). Do not assume it is a firmware issue.
