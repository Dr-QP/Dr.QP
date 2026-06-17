# T2-A · Parse `transferConfig` in `UnixSerial::begin()` instead of hardcoding 8N1

**Severity:** Medium
**Agent turns:** 4
**File:** `packages/runtime/drqp_serial/src/UnixSerial.cpp:89`

## Background

`UnixSerial::begin(uint32_t baudRate, uint8_t transferConfig)` accepts a `transferConfig`
byte but ignores it, hardcoding 8N1:

```cpp
// TODO(anton-matosov): Implement parsing of the options or migrate to explicit options from ext
// SERIAL_8N1
impl_->serial_.set_option(serial_port_base::character_size(8));
impl_->serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));
impl_->serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
```

The silent drop of `transferConfig` creates a hidden contract: every caller must use SERIAL_8N1
or observe the wrong serial framing with no diagnostic. The A1-16 servo does use 8N1, so the
project currently works, but the contract is invisible.

`transferConfig` follows the Arduino `HardwareSerial` bitmask convention:

| Bits | Meaning                                  |
| ---- | ---------------------------------------- |
| 5–4  | Parity: 00=none, 10=even, 11=odd         |
| 3    | Stop bits: 0=one, 1=two                  |
| 2–1  | Data bits offset: 00=5, 01=6, 10=7, 11=8 |

Common values: `SERIAL_8N1 = 0x06`, `SERIAL_8E1 = 0x26`, `SERIAL_8O1 = 0x36`.

## Goal

Either decode the bitmask and apply the correct `boost::asio` options, **or** replace the
`uint8_t transferConfig` API with an explicit options struct and update all call sites. Choose
the approach that fits the rest of the codebase style.

## Acceptance criteria

- [ ] Passing an arbitrary `transferConfig` value produces the corresponding serial options on
      the Boost ASIO port — or the parameter is replaced by an explicit struct and all callers
      updated.
- [ ] 8N1 behaviour is preserved (no regression for existing callers).
- [ ] The TODO comment is removed.
- [ ] A unit test (or an update to an existing test) verifies the mapping for at least 8N1,
      8E1, and 8O1 (can use `SerialPlayer` mock or a gmock on the Boost ASIO option setters).

## Implementation approach

### Turn 1 — Audit call sites and the `ISerial` interface

Find every call to `begin()` across the codebase. Determine whether the interface is defined
in a header shared with `SerialPlayer` and other implementations. Decide: bitmask decode vs.
struct refactor.

### Turn 2 — Implement the chosen approach

**Option A (bitmask decode):**

```cpp
uint8_t dataBits = 5 + ((transferConfig >> 1) & 0x03);
bool twoStopBits = (transferConfig >> 3) & 0x01;
uint8_t parityBits = (transferConfig >> 4) & 0x03;
// map to boost::asio options …
```

**Option B (explicit struct):**

```cpp
struct SerialConfig { uint8_t dataBits = 8; Parity parity = Parity::none; StopBits stopBits = StopBits::one; };
```

Update `ISerial::begin`, `UnixSerial`, `SerialPlayer`, and all call sites.

### Turn 3 — Add/update tests

Write or extend a unit test that exercises at least three `transferConfig` values and asserts
the correct Boost ASIO options are set.

### Turn 4 — Verify build and tests

Build `drqp_serial` and `drqp_a1_16_driver`. Run all serial-related tests.

## Files to modify

| File                                                | Change                           |
| --------------------------------------------------- | -------------------------------- |
| `packages/runtime/drqp_serial/src/UnixSerial.cpp`   | Implement decoding / remove TODO |
| `packages/runtime/drqp_serial/include/…/ISerial.h`  | Update signature (Option B only) |
| `packages/runtime/drqp_serial/src/SerialPlayer.cpp` | Update signature (Option B only) |
| Call sites in `drqp_a1_16_driver`                   | Update (Option B only)           |
| Relevant test file                                  | Add coverage for non-8N1 configs |

## Dependencies

None. Can be done independently.
