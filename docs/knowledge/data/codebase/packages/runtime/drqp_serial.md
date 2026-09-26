---
type: codebase
description: C++ byte-stream transport for the servo bus — Unix UART, TCP, and a record/playback proxy used by hardware-free tests.
source: packages/runtime/drqp_serial
source_digest: sha256:f5860843b9bac265a6c08e7e323814c2338f8da65482d9aca6fe217ab1423d27
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: packages/runtime/drqp_serial
---

# drqp_serial

"A collection of classes to work with UART ports on Unix and via TCP", built on
Boost.Asio. It gives the servo driver an Arduino-style `Stream` /
`SerialProtocol` interface. It can also record a real session to JSON, or replay
one byte-exactly, so the driver tests run without hardware.

## Public surface

- `Stream` and `SerialProtocol`: the abstract byte-stream interfaces
  (`available`, `readBytes`, `writeBytes`, `begin(baud, transferConfig)`).
- `makeSerialForDevice(address)`: builds a transport from an address string.
- `UnixSerial`, `TcpSerial`, `RecordingProxy::SerialRecordingProxy`, and
  `RecordingProxy::SerialPlayer`: the concrete transports.
- `decodeSerialTransferConfig(byte)`: converts the Arduino framing bitmask into
  Boost.Asio options.

## How it works

`makeSerialForDevice` parses an address grammar:

- `playback|<file>`: a `SerialPlayer` that replays a JSON recording.
- A path starting with `/`: a `UnixSerial` opened at 115200 baud.
- Anything else: `host[:port]`, a `TcpSerial` with the port defaulting to 2022.
- A `|<file>` suffix on a real device wraps it in a `SerialRecordingProxy` that
  records the traffic.

The recording format is written and read with `drqp_rapidjson`.

## Depends on

- Boost (Asio).
- `drqp_rapidjson`.

## Invariants & gotchas

- The recording proxy assumes a master/slave UART exchange: a write, followed by
  reads, like HTTP request/response. It cannot record unsolicited traffic.
- A parity field of `01` in the framing byte is reserved and throws
  `std::invalid_argument`.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_serial/include/drqp_serial/Stream.h:27` — `Stream`
- `packages/runtime/drqp_serial/include/drqp_serial/SerialProtocol.h:53` —
  `SerialProtocol`
- `packages/runtime/drqp_serial/src/SerialFactory.cpp:28` —
  `makeSerialForDevice`
- `packages/runtime/drqp_serial/src/SerialFactory.cpp:36` — `playback` address
  branch
- `packages/runtime/drqp_serial/include/drqp_serial/SerialRecordingProxy.h:33` —
  master/slave assumption
- `packages/runtime/drqp_serial/include/drqp_serial/SerialPlayer.h:36` —
  `SerialPlayer`
- `packages/runtime/drqp_serial/include/drqp_serial/SerialTransferConfig.h:51` —
  `decodeSerialTransferConfig`
