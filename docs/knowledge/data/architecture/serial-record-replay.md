---
type: architecture
description: Why servo-protocol tests replay byte-exact serial recordings captured from real hardware, and how TCP and mock transports fit alongside.
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T12:00:00Z
sources:
- id: factory
  resource: packages/runtime/drqp_serial/src/SerialFactory.cpp
- id: tests
  resource: packages/runtime/drqp_a1_16_driver/test/TestXYZrobotServo.cpp
- id: hw
  resource: packages/runtime/drqp_control/src/a1_16_hardware_interface.cpp
---

# Serial record and replay for servo tests

## Decision

The servo bus has three interchangeable transports behind `SerialProtocol`, each
chosen by the device address string (`makeSerialForDevice`):

- **Real:** `/dev/tty…` (UART) or `host[:port]` (TCP, default port 2022). The
  TCP path serves ser2net setups; `scripts/ser2net/` holds a ser2net container.
- **Record:** `<device>|<file.json>` wraps a real transport in
  `SerialRecordingProxy`, which logs every write and read.
- **Replay:** `playback|<file.json>` uses `SerialPlayer`. It asserts that every
  written byte matches the recording and returns the recorded replies.

The `drqp_a1_16_driver` tests run on replay by default. `--use-real-hardware`
(plus `--device`) runs them against real servos and re-records the files in
`test/test_data/`. At a higher level, `drqp_control` swaps the whole servo for
`MockServo` when `device_address` is `mock_servo`.

## Why

It is inferred from the code; no design note records it. Protocol-level driver
tests can run in CI without servos, yet still pin the exact wire bytes that real
A1-16s accepted. The player also requires the recording to be fully consumed, so
a test cannot silently skip traffic.

## Rejected alternatives

*Not recorded.*

## Consequences

- Any change to framing or to a command's byte sequence fails replay until the
  recording is regenerated on real hardware.
- The recording model assumes request/response traffic started by the host. It
  cannot capture unsolicited servo output.
