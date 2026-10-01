---
type: codebase
description: C++ driver for the XYZrobot A1-16 smart servo protocol — EEPROM/RAM access, I-JOG/S-JOG motion, torque and reboot — plus a mock servo.
source: packages/runtime/drqp_a1_16_driver
source_digest: sha256:e5b06990ab089f06962d119d292bfc478df41f8980c32f3eca822b77a7af1d1b
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: packages/runtime/drqp_a1_16_driver
---

# drqp_a1_16_driver

"Driver for A1-16 servo used in Dr.QP robot project." It is derived from
Pololu's XYZrobotServo Arduino library (its license and README are kept
alongside). It talks to one servo, or to all servos through the broadcast ID,
over any `drqp_serial` `Stream`.

## Public surface

- `ServoProtocol`: the abstract per-servo interface that `drqp_control` programs
  against (`readStatus`, `reboot`, `ramRead`, `writeMaxPwmRam`,
  `writeMinMaxPositionRam`, `sendJogCommand`). Its `kBroadcastId` is `0xFE`.
- `XYZrobotServo`: the real implementation. It covers EEPROM and RAM
  reads/writes, ID and baud rate, ACK and LED policy, `setPosition`, `setSpeed`,
  `torqueOn`/`torqueOff`, and `rollback`.
- `MockServo`: an in-memory `ServoProtocol`, selected by `drqp_control` when the
  device address is `mock_servo`.
- `toPlaytime(duration)`: converts a duration into the protocol's 10 ms playtime
  units.
- The `SET_*` jog modes: position, speed, torque-off, and
  position-with-servo-on.
- The `read_everything` example executable dumps a servo's registers.

## How it works

Each command is framed as a protocol packet: the `CMD_*` opcode, the servo ID,
and a checksum. Multi-servo motion uses I-JOG. It sends one packet with an
`IJogData` entry per servo, which the hardware interface fills each ros2_control
cycle.

## Depends on

- [drqp_serial](drqp_serial.md)
- [drqp_interfaces](drqp_interfaces.md)
- `rclcpp`

## Invariants & gotchas

- The tests run against JSON serial recordings in `test/test_data/` unless real
  hardware is requested. The player asserts every recorded byte and requires the
  recording to be fully consumed. A protocol change therefore needs a
  re-recording on real hardware.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_a1_16_driver/include/drqp_a1_16_driver/ServoProtocol.h:143`
  — `ServoProtocol`
- `packages/runtime/drqp_a1_16_driver/include/drqp_a1_16_driver/XYZrobotServo.h:333`
  — `XYZrobotServo`
- `packages/runtime/drqp_a1_16_driver/include/drqp_a1_16_driver/XYZrobotServo.h:144`
  — `toPlaytime`
- `packages/runtime/drqp_a1_16_driver/include/drqp_a1_16_driver/XYZrobotServo.h:149`
  — `SET_*` jog modes
- `packages/runtime/drqp_a1_16_driver/include/drqp_a1_16_driver/MockServo.h:31`
  — `MockServo`
- `packages/runtime/drqp_a1_16_driver/test/TestXYZrobotServo.cpp:218` —
  record-or-replay test fixture
- `packages/runtime/drqp_a1_16_driver/test/TestXYZrobotServo.cpp:247` —
  recording must be fully consumed
