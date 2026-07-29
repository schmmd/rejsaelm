# ELM327 Phase 1 (Pure Logic) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the CAN-reachable ELM327 AT commands that need no hardware, so a generic OBD app completes its handshake against this adapter and gets truthful answers.

**Architecture:** Nearly everything lands in `src/elm327/at_parser.{h,cpp}` and the `AdapterState` struct it owns. Commands that need a value rendered back (protocol description, device identifier, buffer dump) get a pure formatting function in the same unit, called by `session.cpp` when `applyAtCommand` returns the matching `AtResult`. Extended addressing extends `buildSingleFrameRequest` in `src/isotp/request.cpp`. Task 9 is the only one touching `session.cpp`.

**Tech Stack:** C++17, PlatformIO, Unity test framework, `env:native` host tests.

## Global Constraints

- Spec: `docs/superpowers/specs/2026-07-28-elm327-command-set-design.md`. Phase 1 only.
- **`src/elm327/formatter.cpp` and `test/test_formatter/` must not be modified.** `formatPayload()` is a shared wire-format contract with `android/.../ResponseParser.kt`. If a formatter test changes during this work, that is a bug, not a rename.
- **All 74 existing host tests must keep passing** at every commit. Run `pio test -e native`.
- `AdapterState` must stay trivially assignable — `ATZ` resets it with `state = AdapterState{}`. Use POD members only, no `std::string` fields.
- `canonical()` uppercases and strips whitespace. Any command carrying a case-sensitive or space-sensitive payload must parse the **raw** line, not the canonical form.
- Commands this board cannot honestly serve answer `?`. Never `OK` a command whose effect will not happen.
- Follow the existing comment style: explain *why* a decision was made, especially where a naive implementation would be wrong.

---

## File Structure

| File | Responsibility | Change |
|---|---|---|
| `src/elm327/at_parser.h` | `AdapterState`, `AtResult`, command entry points | Modify — new state fields, new results, new pure formatters |
| `src/elm327/at_parser.cpp` | Command dispatch and parsing | Modify |
| `src/isotp/request.h` / `.cpp` | Single-frame request building | Modify — extended addressing |
| `src/elm327/session.cpp` | Renders `AtResult` into reply text | Modify — Task 9 only |
| `test/test_at_parser/test_main.cpp` | Host tests for the parser | Modify — one test per task |
| `test/test_isotp/test_main.cpp` | Host tests for request building | Modify — Task 5 |

---

### Task 1: Recognise the `@` command prefix

`@1`, `@2` and `@3` are ELM327 commands that do **not** start with `AT`. Today `isAtCommand("@1")` returns false, so `@1` falls through to `runRequest()`, fails hex parsing and answers `?`. The existing `at_parser.cpp` checks for `"AT@1"`, which is not a command any client sends.

**Files:**
- Modify: `src/elm327/at_parser.cpp` (`isAtCommand`, and the `AT@1` branch in `applyAtCommand`)
- Test: `test/test_at_parser/test_main.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces: `isAtCommand(const char*)` now returns true for lines beginning `@`. `applyAtCommand` continues to accept both forms.

- [ ] **Step 1: Write the failing test**

```cpp
void test_recognises_the_at_sign_command_prefix() {
    // @1/@2/@3 are ELM327 commands with no AT prefix. Without this, "@1"
    // falls through to runRequest(), fails hex parsing, and answers '?' —
    // which looks to a client exactly like an unsupported command.
    TEST_ASSERT_TRUE(isAtCommand("@1"));
    TEST_ASSERT_TRUE(isAtCommand("@2"));
    TEST_ASSERT_TRUE(isAtCommand(" @3 ABCDEF012345"));
    // A hex request line still must not be mistaken for a command.
    TEST_ASSERT_FALSE(isAtCommand("0100"));
    TEST_ASSERT_FALSE(isAtCommand("220101"));
}
```

Register it in `main()`: `RUN_TEST(test_recognises_the_at_sign_command_prefix);`

- [ ] **Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_at_parser`
Expected: FAIL on the first `TEST_ASSERT_TRUE(isAtCommand("@1"))`.

- [ ] **Step 3: Write minimal implementation**

In `at_parser.cpp`, replace the body of `isAtCommand`:

```cpp
bool isAtCommand(const char* line) {
    std::string s = canonical(line);
    // '@' commands (@1/@2/@3) carry no AT prefix — see applyAtCommand.
    return startsWith(s, "AT") || startsWith(s, "@");
}
```

And in `applyAtCommand`, replace the early bail and the `AT@1` branch:

```cpp
AtResult applyAtCommand(const char* line, AdapterState& state) {
    std::string s = canonical(line);
    if (!startsWith(s, "AT") && !startsWith(s, "@")) return AtResult::Unknown;
```

Leave `if (s == "ATI" || s == "AT@1") return AtResult::Identify;` as
`if (s == "ATI") return AtResult::Identify;` — `@1` gets its own result in Task 2.

- [ ] **Step 4: Run tests to verify they pass**

Run: `pio test -e native`
Expected: PASS, all tests including the previously existing ones.

- [ ] **Step 5: Commit**

```bash
git add src/elm327/at_parser.cpp test/test_at_parser/test_main.cpp
git commit -m "Recognise the @ command prefix.

@1/@2/@3 carry no AT prefix, so isAtCommand rejected them and they fell
through to runRequest() to fail hex parsing. The AT@1 form that was matched
instead is not a command any client sends."
```

---

### Task 2: `@1`, `@2`, `@3` — device description and identifier

`@1` is a device *description* and is distinct from `ATI`'s version banner; conflating them was the pre-existing bug. `@2` reports a user-set identifier, `@3` sets it.

**The identifier must be parsed from the raw line.** `canonical()` uppercases and strips spaces, which would silently mangle the value.

**Files:**
- Modify: `src/elm327/at_parser.h`, `src/elm327/at_parser.cpp`
- Test: `test/test_at_parser/test_main.cpp`

**Interfaces:**
- Consumes: Task 1's `@` prefix recognition.
- Produces: `AdapterState::identifier` (`char[13]`), `AtResult::DeviceDescription`, `AtResult::DeviceIdentifier`.

- [ ] **Step 1: Write the failing test**

```cpp
void test_device_identifier_round_trips_verbatim() {
    AdapterState s;
    // Unset, @2 has nothing to report. '?' is the honest answer, not "".
    TEST_ASSERT_EQUAL(AtResult::Unknown, applyAtCommand("@2", s));

    // Exactly 12 characters, stored verbatim: canonical() would uppercase
    // and strip spaces, which is fine for commands and wrong for a payload.
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("@3 RejsaElm001", s));
    TEST_ASSERT_EQUAL(AtResult::DeviceIdentifier, applyAtCommand("@2", s));
    TEST_ASSERT_EQUAL_STRING("RejsaElm001", s.identifier);

    // Wrong length is rejected, and must not partially overwrite.
    TEST_ASSERT_EQUAL(AtResult::Unknown, applyAtCommand("@3 SHORT", s));
    TEST_ASSERT_EQUAL_STRING("RejsaElm001", s.identifier);
}

void test_device_description_is_not_the_version_banner() {
    AdapterState s;
    // @1 is a device description; ATI is the version banner. A client that
    // probes both and gets one string twice cannot tell them apart.
    TEST_ASSERT_EQUAL(AtResult::DeviceDescription, applyAtCommand("@1", s));
    TEST_ASSERT_EQUAL(AtResult::Identify, applyAtCommand("ATI", s));
}

void test_reset_clears_the_device_identifier() {
    AdapterState s;
    applyAtCommand("@3 RejsaElm001", s);
    TEST_ASSERT_EQUAL(AtResult::Reset, applyAtCommand("ATZ", s));
    TEST_ASSERT_EQUAL_STRING("", s.identifier);
}
```

Register all three in `main()`.

- [ ] **Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_at_parser`
Expected: FAIL — `AtResult::DeviceDescription` does not exist (compile error).

- [ ] **Step 3: Write minimal implementation**

In `at_parser.h`, add to `AdapterState`:

```cpp
    // @3 — a client-set device identifier, exactly 12 characters, reported
    // back by @2. Stored raw: this is a payload, not a command, so the
    // uppercasing and space-stripping canonical() does must not touch it.
    // A char array rather than std::string keeps AdapterState trivially
    // assignable, which is what makes `state = AdapterState{}` a valid reset.
    char identifier[13] = {};
```

Add to `AtResult`:

```cpp
    DeviceDescription,  // @1 — answer with the device description
    DeviceIdentifier,   // @2 — answer with state.identifier
```

In `at_parser.cpp`, add near the top of `applyAtCommand` (after the prefix check):

```cpp
    if (s == "@1") return AtResult::DeviceDescription;
    if (s == "@2") {
        // Nothing has been set, so there is nothing truthful to report.
        return state.identifier[0] ? AtResult::DeviceIdentifier
                                   : AtResult::Unknown;
    }
    if (startsWith(s, "@3")) {
        // Parse the RAW line: the identifier is a payload and canonical()
        // has already destroyed its case and spacing.
        const char* p = std::strchr(line, '3');
        if (!p) return AtResult::Unknown;
        ++p;
        while (*p == ' ') ++p;
        const size_t len = std::strlen(p);
        // ELM327 specifies exactly 12 characters. Accepting a short value
        // would leave the rest of the field as stale bytes from a previous set.
        if (len != 12) return AtResult::Unknown;
        std::memcpy(state.identifier, p, 12);
        state.identifier[12] = '\0';
        return AtResult::Ok;
    }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pio test -e native`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/elm327/at_parser.h src/elm327/at_parser.cpp test/test_at_parser/test_main.cpp
git commit -m "Add @1 device description and @2/@3 device identifier.

@1 previously returned the ATI version banner, so a client probing both
could not tell them apart. The identifier is parsed from the raw line
because canonical() uppercases and strips spaces."
```

---

### Task 3: `ATDP` / `ATDPN` — describe the protocol

Almost every generic OBD app calls these during handshake. Phase 1 speaks exactly one protocol; phase 3 extends the description table.

**Files:**
- Modify: `src/elm327/at_parser.h`, `src/elm327/at_parser.cpp`
- Test: `test/test_at_parser/test_main.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces: `AdapterState::protocol` (`uint8_t`), `AdapterState::autoSelected` (`bool`), `AtResult::DescribeProtocol`, `AtResult::DescribeProtocolNumber`, and two pure formatters:
  - `std::string describeProtocol(const AdapterState&)`
  - `std::string describeProtocolNumber(const AdapterState&)`

- [ ] **Step 1: Write the failing test**

```cpp
void test_describes_the_current_protocol() {
    AdapterState s;
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATSP6", s));
    TEST_ASSERT_EQUAL(AtResult::DescribeProtocol, applyAtCommand("ATDP", s));
    TEST_ASSERT_EQUAL_STRING("ISO 15765-4 (CAN 11/500)",
                             describeProtocol(s).c_str());
    TEST_ASSERT_EQUAL(AtResult::DescribeProtocolNumber,
                      applyAtCommand("ATDPN", s));
    TEST_ASSERT_EQUAL_STRING("6", describeProtocolNumber(s).c_str());
}

void test_auto_selected_protocol_is_reported_as_auto() {
    AdapterState s;
    // ATSP0 asks the adapter to choose. A real ELM327 then reports the
    // choice with an "A" marker, so a client can tell a negotiated protocol
    // from one it pinned itself.
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATSP0", s));
    TEST_ASSERT_EQUAL_STRING("AUTO, ISO 15765-4 (CAN 11/500)",
                             describeProtocol(s).c_str());
    TEST_ASSERT_EQUAL_STRING("A6", describeProtocolNumber(s).c_str());

    // ATSPA6 is "auto, starting at 6" — also auto.
    AdapterState a;
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATSPA6", a));
    TEST_ASSERT_EQUAL_STRING("A6", describeProtocolNumber(a).c_str());

    // ATSP6 pins it, so no marker.
    AdapterState p;
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATSP6", p));
    TEST_ASSERT_EQUAL_STRING("6", describeProtocolNumber(p).c_str());
}
```

Register both in `main()`.

- [ ] **Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_at_parser`
Expected: FAIL — `describeProtocol` not declared (compile error).

- [ ] **Step 3: Write minimal implementation**

In `at_parser.h`, add `#include <string>`, then add to `AdapterState`:

```cpp
    // ATSP — the selected protocol number. Only 6 (ISO 15765-4, CAN 11-bit,
    // 500 kbit/s) is selectable until phase 3 adds 7/8/9.
    uint8_t protocol = 6;
    // True when the client asked the adapter to choose (ATSP0 / ATSPAn)
    // rather than pinning one. ATDP/ATDPN mark this, so a client can tell a
    // negotiated protocol from one it set itself.
    bool autoSelected = false;
```

Add to `AtResult`:

```cpp
    DescribeProtocol,        // ATDP  — answer with describeProtocol(state)
    DescribeProtocolNumber,  // ATDPN — answer with describeProtocolNumber(state)
```

Declare at the bottom of `at_parser.h`:

```cpp
// ATDP / ATDPN renderings. Pure functions of the state so they are host-tested
// rather than living as string literals inside session.cpp.
std::string describeProtocol(const AdapterState& state);
std::string describeProtocolNumber(const AdapterState& state);
```

In `at_parser.cpp`, replace the `ATSP` branch:

```cpp
    if (startsWith(s, "ATSP")) {
        // We speak exactly one protocol: ISO 15765-4 CAN, 11-bit, 500 kbit/s
        // (protocol 6). Phase 3 adds 7/8/9. Anything else is rejected rather
        // than accepted and silently not driven.
        if (s == "ATSP6")   { state.protocol = 6; state.autoSelected = false; return AtResult::Ok; }
        if (s == "ATSP0")   { state.protocol = 6; state.autoSelected = true;  return AtResult::Ok; }
        if (s == "ATSPA6" || s == "ATSPA0") {
            state.protocol = 6; state.autoSelected = true; return AtResult::Ok;
        }
        return AtResult::Unknown;
    }

    if (s == "ATDP")  return AtResult::DescribeProtocol;
    if (s == "ATDPN") return AtResult::DescribeProtocolNumber;
```

Note: place the `ATDP`/`ATDPN` checks **before** the `ATSP` branch is irrelevant (different prefixes), but they must come before the catch-all `return AtResult::Unknown;`.

Add the formatters at the end of `at_parser.cpp`:

```cpp
std::string describeProtocol(const AdapterState& state) {
    // One protocol until phase 3 adds 7/8/9; a switch here then.
    std::string out = state.autoSelected ? "AUTO, " : "";
    out += "ISO 15765-4 (CAN 11/500)";
    return out;
}

std::string describeProtocolNumber(const AdapterState& state) {
    std::string out = state.autoSelected ? "A" : "";
    out += static_cast<char>('0' + state.protocol);
    return out;
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pio test -e native`
Expected: PASS. The pre-existing `test_protocol_select_is_accepted` still passes — `ATSP6`, `ATSP0` still `Ok`, `ATSP3` still `Unknown`.

- [ ] **Step 5: Commit**

```bash
git add src/elm327/at_parser.h src/elm327/at_parser.cpp test/test_at_parser/test_main.cpp
git commit -m "Add ATDP and ATDPN protocol description.

Generic OBD apps call these during handshake. The renderings are pure
functions of the state so they are host-tested rather than string literals
buried in session.cpp."
```

---

### Task 4: Stored request-shaping flags — `ATR0/R1`, `ATV0/V1`, `ATTA`, `ATCP`, `ATAL/ATNL`

Five commands that only set state here. Task 9 wires the first two into `session.cpp`. `ATTA` and `ATCP` are stored for phase 3. `ATAL`/`ATNL` are **stored and not honoured** — `runRequest()` builds single frames only, so there is no multi-frame transmit to enable; say so rather than implying an effect.

**Files:**
- Modify: `src/elm327/at_parser.h`, `src/elm327/at_parser.cpp`
- Test: `test/test_at_parser/test_main.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces: `AdapterState::responses`, `::variableDlc`, `::allowLong`, `::testerAddress`, `::priorityBits`.

- [ ] **Step 1: Write the failing test**

```cpp
void test_request_shaping_flags_are_stored() {
    AdapterState s;
    TEST_ASSERT_TRUE(s.responses);      // R1 is the default
    TEST_ASSERT_FALSE(s.variableDlc);   // V0 is the default: always DLC 8

    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATR0", s));
    TEST_ASSERT_FALSE(s.responses);
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATR1", s));
    TEST_ASSERT_TRUE(s.responses);

    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATV1", s));
    TEST_ASSERT_TRUE(s.variableDlc);
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATV0", s));
    TEST_ASSERT_FALSE(s.variableDlc);

    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATAL", s));
    TEST_ASSERT_TRUE(s.allowLong);
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATNL", s));
    TEST_ASSERT_FALSE(s.allowLong);

    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATTA F9", s));
    TEST_ASSERT_EQUAL_UINT8(0xF9, s.testerAddress);
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATCP18", s));
    TEST_ASSERT_EQUAL_UINT8(0x18, s.priorityBits);
}

void test_byte_valued_commands_reject_out_of_range_and_garbage() {
    AdapterState s;
    const uint8_t ta = s.testerAddress;
    // A value wider than one byte is not a tester address; truncating it
    // would address something the client never named.
    TEST_ASSERT_EQUAL(AtResult::Unknown, applyAtCommand("ATTA100", s));
    TEST_ASSERT_EQUAL(AtResult::Unknown, applyAtCommand("ATTAZZ", s));
    TEST_ASSERT_EQUAL(AtResult::Unknown, applyAtCommand("ATTA", s));
    TEST_ASSERT_EQUAL_UINT8(ta, s.testerAddress);
}
```

Register both in `main()`.

- [ ] **Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_at_parser`
Expected: FAIL — `s.responses` does not exist (compile error).

- [ ] **Step 3: Write minimal implementation**

In `at_parser.h`, add to `AdapterState`:

```cpp
    bool responses = true;      // ATR1 — wait for a reply after transmitting
    bool variableDlc = false;   // ATV0 — send full 8-byte frames
    // ATAL/ATNL — STORED, NOT HONOURED. Allowing messages longer than 7 bytes
    // means multi-frame transmit, and runRequest() builds single frames only.
    // There is nothing to enable, so this flag changes no behaviour today.
    bool allowLong = false;
    uint8_t testerAddress = 0xF9;  // ATTA — the conventional OBD tester address
    uint8_t priorityBits = 0x18;   // ATCP — 29-bit ID priority, phase 3
```

Add a byte-parsing helper in the anonymous namespace of `at_parser.cpp`, beside `hexTail`:

```cpp
// Parses a one-byte hex tail, e.g. ATTAF9. Rejects values that do not fit in
// a byte rather than truncating them into a different address.
bool byteTail(const std::string& s, size_t from, uint8_t& value) {
    uint32_t v = 0;
    if (!hexTail(s, from, v) || v > 0xFF) return false;
    value = static_cast<uint8_t>(v);
    return true;
}
```

Add the branches in `applyAtCommand`, before the catch-all:

```cpp
    if (s == "ATR0") { state.responses = false; return AtResult::Ok; }
    if (s == "ATR1") { state.responses = true;  return AtResult::Ok; }
    if (s == "ATV0") { state.variableDlc = false; return AtResult::Ok; }
    if (s == "ATV1") { state.variableDlc = true;  return AtResult::Ok; }
    if (s == "ATAL") { state.allowLong = true;  return AtResult::Ok; }
    if (s == "ATNL") { state.allowLong = false; return AtResult::Ok; }

    if (startsWith(s, "ATTA")) {
        uint8_t v = 0;
        if (!byteTail(s, 4, v)) return AtResult::Unknown;
        state.testerAddress = v;
        return AtResult::Ok;
    }
    if (startsWith(s, "ATCP")) {
        uint8_t v = 0;
        if (!byteTail(s, 4, v)) return AtResult::Unknown;
        state.priorityBits = v;
        return AtResult::Ok;
    }
```

**Ordering note:** `ATCP` must be tested before any `startsWith(s, "ATC…")` catch-all. The existing harmless list matches `ATCF`, `ATCM`, `ATCRA` exactly, so there is no conflict — but keep `ATCP` above them.

- [ ] **Step 4: Run tests to verify they pass**

Run: `pio test -e native`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/elm327/at_parser.h src/elm327/at_parser.cpp test/test_at_parser/test_main.cpp
git commit -m "Add ATR0/R1, ATV0/V1, ATAL/ATNL, ATTA and ATCP state.

ATAL/ATNL are stored and not honoured: multi-frame transmit does not exist,
so there is nothing for them to enable. Documented rather than implied."
```

---

### Task 5: `ATCEA` — CAN extended addressing

Extended addressing prepends an address byte to every frame, which costs one byte of payload. `buildSingleFrameRequest` is host-tested, so this lands as a real, tested behaviour change rather than a stored flag.

**Files:**
- Modify: `src/isotp/request.h`, `src/isotp/request.cpp`
- Modify: `src/elm327/at_parser.h`, `src/elm327/at_parser.cpp`
- Test: `test/test_isotp/test_main.cpp`, `test/test_at_parser/test_main.cpp`

**Interfaces:**
- Consumes: `byteTail()` from Task 4.
- Produces: `AdapterState::extendedAddressing` (`bool`), `::extendedAddress` (`uint8_t`); new signature
  `bool buildSingleFrameRequest(const uint8_t* payload, size_t len, uint8_t out[8], bool extendedAddressing = false, uint8_t extendedAddress = 0)`.

- [ ] **Step 1: Write the failing tests**

In `test/test_isotp/test_main.cpp`:

```cpp
void test_extended_addressing_prepends_the_address_byte() {
    uint8_t out[8] = {};
    const uint8_t payload[] = {0x22, 0x01, 0x01};
    TEST_ASSERT_TRUE(buildSingleFrameRequest(payload, 3, out, true, 0xF1));
    // Address byte, then the normal single-frame PCI and payload.
    TEST_ASSERT_EQUAL_UINT8(0xF1, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0x03, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0x22, out[2]);
    TEST_ASSERT_EQUAL_UINT8(0x01, out[3]);
    TEST_ASSERT_EQUAL_UINT8(0x01, out[4]);
}

void test_extended_addressing_costs_one_byte_of_payload() {
    uint8_t out[8] = {};
    const uint8_t seven[] = {1, 2, 3, 4, 5, 6, 7};
    // Seven bytes fit normally...
    TEST_ASSERT_TRUE(buildSingleFrameRequest(seven, 7, out));
    // ...but not once an address byte is taking a slot. Truncating here would
    // send a shorter request than the client asked for, and the reply would
    // decode as a different DID.
    TEST_ASSERT_FALSE(buildSingleFrameRequest(seven, 7, out, true, 0xF1));
    const uint8_t six[] = {1, 2, 3, 4, 5, 6};
    TEST_ASSERT_TRUE(buildSingleFrameRequest(six, 6, out, true, 0xF1));
}

void test_default_arguments_leave_existing_behaviour_unchanged() {
    uint8_t with_defaults[8] = {};
    uint8_t explicit_off[8] = {};
    const uint8_t payload[] = {0x22, 0x01, 0x01};
    TEST_ASSERT_TRUE(buildSingleFrameRequest(payload, 3, with_defaults));
    TEST_ASSERT_TRUE(buildSingleFrameRequest(payload, 3, explicit_off, false, 0));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(with_defaults, explicit_off, 8);
}
```

In `test/test_at_parser/test_main.cpp`:

```cpp
void test_extended_addressing_is_set_and_cleared() {
    AdapterState s;
    TEST_ASSERT_FALSE(s.extendedAddressing);
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATCEA F1", s));
    TEST_ASSERT_TRUE(s.extendedAddressing);
    TEST_ASSERT_EQUAL_UINT8(0xF1, s.extendedAddress);
    // A bare ATCEA turns extended addressing back off.
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATCEA", s));
    TEST_ASSERT_FALSE(s.extendedAddressing);
}
```

Register all four in their respective `main()` functions.

- [ ] **Step 2: Run tests to verify they fail**

Run: `pio test -e native`
Expected: FAIL — `buildSingleFrameRequest` takes 3 arguments (compile error).

- [ ] **Step 3: Write minimal implementation**

In `src/isotp/request.h`:

```cpp
// Builds a single-frame ISO-TP request. Returns false if the payload is empty
// or does not fit — multi-frame requests are not needed here, and silently
// truncating one would send a different request than was asked for.
//
// With extended addressing (ATCEA) the address byte occupies out[0], so the
// usable payload drops from 7 bytes to 6. The default arguments keep every
// existing caller building exactly the frame it built before.
bool buildSingleFrameRequest(const uint8_t* payload, size_t len, uint8_t out[8],
                             bool extendedAddressing = false,
                             uint8_t extendedAddress = 0);
```

In `src/isotp/request.cpp`, adapt the existing body — write the address byte first when enabled, then shift the PCI and payload by one, and cap the length at `extendedAddressing ? 6 : 7`.

In `at_parser.h`, add to `AdapterState`:

```cpp
    // ATCEA — CAN extended addressing. The address byte occupies the first
    // data byte of every frame, so the usable payload drops to 6.
    bool extendedAddressing = false;
    uint8_t extendedAddress = 0;
```

In `at_parser.cpp`, before the harmless catch-all (and before any `ATC…` prefix match):

```cpp
    if (startsWith(s, "ATCEA")) {
        // A bare ATCEA turns extended addressing off.
        if (s == "ATCEA") { state.extendedAddressing = false; return AtResult::Ok; }
        uint8_t v = 0;
        if (!byteTail(s, 5, v)) return AtResult::Unknown;
        state.extendedAddressing = true;
        state.extendedAddress = v;
        return AtResult::Ok;
    }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pio test -e native`
Expected: PASS, including every pre-existing `test_isotp` case unchanged.

- [ ] **Step 5: Commit**

```bash
git add src/isotp/request.h src/isotp/request.cpp src/elm327/at_parser.h \
        src/elm327/at_parser.cpp test/test_isotp/test_main.cpp \
        test/test_at_parser/test_main.cpp
git commit -m "Add ATCEA CAN extended addressing.

The address byte takes the first data slot, so the usable payload drops to
six. Default arguments keep existing callers byte-identical."
```

---

### Task 6: `ATBD` — buffer dump

Reports the last received frame. The *rendering* is a pure function over state and is host-tested here; `session.cpp` populating `lastFrame` is Task 9.

**Files:**
- Modify: `src/elm327/at_parser.h`, `src/elm327/at_parser.cpp`
- Test: `test/test_at_parser/test_main.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces: `AdapterState::lastFrame` (a `ReceivedFrame` POD), `AtResult::BufferDump`, `std::string formatBufferDump(const AdapterState&)`.

- [ ] **Step 1: Write the failing test**

```cpp
void test_buffer_dump_renders_the_last_received_frame() {
    AdapterState s;
    // Nothing received yet — there is no buffer to dump.
    TEST_ASSERT_EQUAL(AtResult::Unknown, applyAtCommand("ATBD", s));

    s.lastFrame.valid = true;
    s.lastFrame.dlc = 8;
    const uint8_t data[8] = {0x03, 0x41, 0x0C, 0x1A, 0xF8, 0x00, 0x00, 0x00};
    std::memcpy(s.lastFrame.data, data, 8);

    TEST_ASSERT_EQUAL(AtResult::BufferDump, applyAtCommand("ATBD", s));
    // Length first, then the bytes — the ELM327 rendering.
    TEST_ASSERT_EQUAL_STRING("08 03 41 0C 1A F8 00 00 00",
                             formatBufferDump(s).c_str());
}

void test_buffer_dump_honours_the_frame_length() {
    AdapterState s;
    s.lastFrame.valid = true;
    s.lastFrame.dlc = 3;
    const uint8_t data[8] = {0xAA, 0xBB, 0xCC, 0, 0, 0, 0, 0};
    std::memcpy(s.lastFrame.data, data, 8);
    // Bytes beyond the DLC are not part of the frame and must not be printed
    // as though the ECU sent them.
    TEST_ASSERT_EQUAL_STRING("03 AA BB CC", formatBufferDump(s).c_str());
}
```

Register both in `main()`.

- [ ] **Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_at_parser`
Expected: FAIL — `s.lastFrame` does not exist (compile error).

- [ ] **Step 3: Write minimal implementation**

In `at_parser.h`, above `AdapterState`:

```cpp
// The most recent frame accepted by the session, kept solely so ATBD has
// something to report. One frame, not the whole sequence: ATBD dumps a buffer,
// and retaining a full multi-frame response would be the frame-retention work
// that real ATH1 support needs and this phase deliberately does not do.
struct ReceivedFrame {
    bool valid = false;
    uint8_t dlc = 0;
    uint8_t data[8] = {};
};
```

Add to `AdapterState`: `ReceivedFrame lastFrame;`

Add to `AtResult`: `BufferDump,  // ATBD — answer with formatBufferDump(state)`

Declare: `std::string formatBufferDump(const AdapterState& state);`

In `at_parser.cpp`, add the branch:

```cpp
    if (s == "ATBD") {
        return state.lastFrame.valid ? AtResult::BufferDump : AtResult::Unknown;
    }
```

And the formatter:

```cpp
std::string formatBufferDump(const AdapterState& state) {
    char buf[4];
    std::snprintf(buf, sizeof(buf), "%02X", state.lastFrame.dlc);
    std::string out = buf;
    // Only up to the DLC: the tail of the array is whatever was there before,
    // not something the ECU sent.
    for (uint8_t i = 0; i < state.lastFrame.dlc && i < 8; ++i) {
        std::snprintf(buf, sizeof(buf), "%02X", state.lastFrame.data[i]);
        out += ' ';
        out += buf;
    }
    return out;
}
```

Add `#include <cstdio>` to `at_parser.cpp` if not present.

- [ ] **Step 4: Run tests to verify they pass**

Run: `pio test -e native`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/elm327/at_parser.h src/elm327/at_parser.cpp test/test_at_parser/test_main.cpp
git commit -m "Add ATBD buffer dump.

One frame only. Retaining a full multi-frame response is the frame-retention
work real ATH1 needs, which this phase deliberately does not do."
```

---

### Task 7: `ATPC` and the monitor-mode stubs

`ATPC` closes the protocol. `ATMA`/`ATMR`/`ATMT` answer `NO DATA` immediately — not because monitoring is impossible on this board, but because it is not built until phase 4. Answering immediately beats making a client wait out a full timeout for a reply that is not coming.

**Files:**
- Modify: `src/elm327/at_parser.h`, `src/elm327/at_parser.cpp`
- Test: `test/test_at_parser/test_main.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces: `AtResult::NoData`.

- [ ] **Step 1: Write the failing test**

```cpp
void test_monitor_commands_report_no_data_until_phase_four() {
    AdapterState s;
    // Streaming monitor modes arrive in phase 4. Until then answer at once
    // rather than making the client wait out a timeout. NO DATA is the
    // honest answer: nothing was captured.
    TEST_ASSERT_EQUAL(AtResult::NoData, applyAtCommand("ATMA", s));
    TEST_ASSERT_EQUAL(AtResult::NoData, applyAtCommand("ATMR 7E8", s));
    TEST_ASSERT_EQUAL(AtResult::NoData, applyAtCommand("ATMT 7E0", s));
}

void test_protocol_close_is_accepted() {
    AdapterState s;
    TEST_ASSERT_EQUAL(AtResult::Ok, applyAtCommand("ATPC", s));
}
```

Register both in `main()`.

- [ ] **Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_at_parser`
Expected: FAIL — `AtResult::NoData` does not exist (compile error).

- [ ] **Step 3: Write minimal implementation**

Add to `AtResult`: `NoData,  // answer "NO DATA"`

In `at_parser.cpp`:

```cpp
    if (s == "ATPC") return AtResult::Ok;

    // Monitor modes stream frames until interrupted, which the session cannot
    // do yet — handleLine() is strictly request/reply/prompt. Phase 4 builds
    // the streaming path and these become real. Answer immediately meanwhile.
    if (s == "ATMA" || startsWith(s, "ATMR") || startsWith(s, "ATMT")) {
        return AtResult::NoData;
    }
```

**Ordering note:** these must come after the `ATM0`/`ATM1` entries in the harmless list, or `ATM0` will not match. Verify `test_harmless_commands_are_accepted_without_effect` still passes — it asserts `ATM0` returns `Ok`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `pio test -e native`
Expected: PASS, including the pre-existing `ATM0` assertion.

- [ ] **Step 5: Commit**

```bash
git add src/elm327/at_parser.h src/elm327/at_parser.cpp test/test_at_parser/test_main.cpp
git commit -m "Add ATPC and stub the monitor modes with NO DATA.

Monitoring needs the streaming session from phase 4. Answering at once beats
making a client wait out a timeout for a reply that is not coming."
```

---

### Task 8: Pin the refusal pile

The commands that must answer `?`. This test is the point of the task: it is what stops a later refactor from quietly turning a refusal into an `OK`, which would leave a client waiting on a bus this board cannot drive.

**Files:**
- Test: `test/test_at_parser/test_main.cpp`
- Modify: `src/elm327/at_parser.cpp` only if a command is found to be wrongly accepted.

**Interfaces:**
- Consumes: every prior task.
- Produces: nothing.

- [ ] **Step 1: Write the test**

```cpp
void test_commands_for_buses_this_board_cannot_drive_are_refused() {
    // These serve J1850, ISO 9141 and ISO 14230 — protocols with no
    // electrical path on a CAN-only board. Answering OK would leave a client
    // waiting on a bus that will never reply, which is a worse failure than
    // an honest '?'. Phase 4 moves the J1939 entries out of this list.
    const char* refused[] = {
        "ATIB10", "ATIB96", "ATFI", "ATSI", "ATKW0", "ATKW1", "ATBI",
        "ATSW20", "ATWM8106F1", "ATSP1", "ATSP2", "ATSP3", "ATSP4", "ATSP5",
        "ATJE", "ATJS", "ATJHF0", "ATJHF1", "ATJTM1", "ATJTM5",
        "ATMP1234", "ATDM1",
    };
    for (const char* command : refused) {
        AdapterState s;
        const AdapterState before = s;
        TEST_ASSERT_EQUAL(AtResult::Unknown, applyAtCommand(command, s));
        // A refusal must also not have moved anything on the way out.
        TEST_ASSERT_EQUAL_UINT16(before.header, s.header);
        TEST_ASSERT_EQUAL_UINT16(before.timeoutMs, s.timeoutMs);
        TEST_ASSERT_EQUAL_UINT8(before.protocol, s.protocol);
    }
}
```

Register it in `main()`.

- [ ] **Step 2: Run the test**

Run: `pio test -e native -f test_at_parser`
Expected: PASS if no prior task over-matched. If any entry fails, a prefix
match from an earlier task is too greedy — narrow it, do not delete the
assertion.

- [ ] **Step 3: Fix any over-matching**

Most likely culprits: `startsWith(s, "ATM…")` from Task 7 swallowing `ATMP`,
and the harmless `startsWith(s, "ATCF")` family. `ATMP` must be refused, not
treated as `ATMT`. Narrow the Task 7 checks to exact matches plus a hex tail if
this fires.

- [ ] **Step 4: Run the full suite**

Run: `pio test -e native`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/elm327/at_parser.cpp test/test_at_parser/test_main.cpp
git commit -m "Pin the commands that must answer '?'.

A CAN-only board cannot serve J1850, ISO 9141 or ISO 14230. This test is what
stops a later refactor from turning a refusal into an OK and leaving a client
waiting on a bus that will never reply."
```

---

### Task 9: Wire the new results into the session

The only task touching `session.cpp`, and the only one not covered by host tests — `session.cpp` is excluded from `env:native` because it needs `Arduino.h` and the TWAI driver.

**Files:**
- Modify: `src/elm327/session.cpp`
- Modify: `tools/bringup_checklist.md`

**Interfaces:**
- Consumes: every `AtResult` and formatter from Tasks 2–7.
- Produces: nothing.

- [ ] **Step 1: Render the new results**

In `handleLine()`, extend the switch:

```cpp
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
```

Add beside `kVersionBanner`:

```cpp
// @1 — a device description, distinct from the ATI version banner.
const char* kDeviceDescription = "RejsaElm OBD-II Adapter";
```

- [ ] **Step 2: Honour `ATR0` and `ATV0/V1` in `runRequest()`**

Set the DLC from `variableDlc`, replacing the hardcoded `data_length_code = 8`:

```cpp
    // ATV1 sends only the bytes used; ATV0 (the default) pads to 8, which is
    // what almost every ECU expects.
    request.data_length_code = state_.variableDlc
        ? static_cast<uint8_t>(state_.extendedAddressing ? payloadLen + 2 : payloadLen + 1)
        : 8;
```

Pass extended addressing through to the builder:

```cpp
    if (!buildSingleFrameRequest(payload, payloadLen, request.data,
                                 state_.extendedAddressing,
                                 state_.extendedAddress)) {
        return formatFault("?");
    }
```

And return immediately when responses are off, after the transmit succeeds and
before the receive loop:

```cpp
    // ATR0: fire and forget. The client has said it does not want a reply, so
    // waiting out the full timeout would only stall the next command.
    if (!state_.responses) return "OK";
```

- [ ] **Step 3: Populate `lastFrame` for `ATBD`**

Inside the receive loop, immediately after a frame passes the `addressed` check:

```cpp
        // Keep the newest accepted frame so ATBD has something to report.
        state_.lastFrame.valid = true;
        state_.lastFrame.dlc = rx.data_length_code;
        std::memcpy(state_.lastFrame.data, rx.data, 8);
```

Add `#include <cstring>` to `session.cpp`.

- [ ] **Step 4: Verify it builds for the board and the host**

Run: `pio run -e rejsacan`
Expected: builds clean, no warnings about unhandled switch cases.

Run: `pio test -e native`
Expected: PASS, all tests.

- [ ] **Step 5: Add the unverified entries to the bringup checklist**

Append to `tools/bringup_checklist.md` a row per unverified behaviour: `ATDP`,
`ATDPN`, `@1`, `@2`/`@3`, `ATBD` against a real frame, `ATR0` fire-and-forget,
`ATV1` variable DLC, `ATCEA` against an ECU that uses extended addressing.
Mark each unverified, consistent with the existing rows.

- [ ] **Step 6: Commit**

```bash
git add src/elm327/session.cpp tools/bringup_checklist.md
git commit -m "Wire the phase 1 AT results into the session.

Renders the new results, honours ATR0 and ATV0/V1 in runRequest(), and keeps
the newest accepted frame for ATBD. Not host-tested — session.cpp needs
Arduino.h and the TWAI driver — so the new behaviours go to the checklist."
```

---

## Self-Review

**Spec coverage.** Every phase 1 row of the spec table maps to a task: `ATDP`/`ATDPN` → 3; `ATR0`/`R1`, `ATV0`/`V1` → 4 and 9; `ATCEA` → 5; `ATBD` → 6 and 9; `ATTA`, `ATCP` → 4; `ATPC` → 7; `@1`/`@2`/`@3` → 1 and 2; `ATAL`/`ATNL` → 4; `ATMA`/`ATMR`/`ATMT` → 7; the refusal pile → 8. The spec's note that `@1` currently returns the `ATI` banner is fixed in Task 2.

**Type consistency.** `AdapterState` gains `identifier`, `protocol`, `autoSelected`, `responses`, `variableDlc`, `allowLong`, `testerAddress`, `priorityBits`, `extendedAddressing`, `extendedAddress`, `lastFrame` — each defined in exactly one task and used with the same name and type afterwards. `AtResult` gains `DeviceDescription`, `DeviceIdentifier`, `DescribeProtocol`, `DescribeProtocolNumber`, `BufferDump`, `NoData`, all handled in Task 9's switch.

**Known gap, deliberately left.** Task 8's refusal test is the one most likely to fail on first run, because Task 7's `startsWith(s, "ATM…")` checks can swallow `ATMP`. Task 8 Step 3 names that specific failure and the fix. Ordering hazards are called out inline in Tasks 4 and 7 rather than left to be discovered.
