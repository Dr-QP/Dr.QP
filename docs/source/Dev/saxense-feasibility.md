# SAxense feasibility for DualSense audio and haptics

This spike is a desk-research assessment of SAxense as a possible optional
backend for DualSense feedback in Dr.QP, conducted with full internet access.
It does not include target-hardware measurements, because the CI environment
does not provide a Raspberry Pi or a paired DualSense controller.

## Executive summary

- **Decision:** NO GO. While it handles haptics, it doesn't handle audio. Even for
  Haptics it is a very rough POC which is brittle. The proper HIDAPI layer needs
  to be built instead.
- **Why:** SAxense is the only known open-source tool that enables DualSense
  **haptics over Bluetooth** on Linux. Every alternative (including `pydualsense`)
  is limited to USB for haptic output. Because Dr.QP connects to the controller
  over Bluetooth, SAxense is the correct tool for any Bluetooth haptics path.
- **Constraints:** The project is a young proof-of-concept (created July 2025,
  two commits), requires a C build step and `ffmpeg`, and has no measured
  target-device latency data. Integration must be fully optional with clean
  fallback.
- **Next step:** Validated target-hardware measurements (latency, reconnect,
  resource use) are required before any code lands in a release build. The
  rumble-only path from
  [#241](https://github.com/Dr-QP/Dr.QP/issues/241) remains the supported
  baseline until then.

## What SAxense actually does

SAxense is a ~100-line C program
([source](https://github.com/egormanga/SAxense/blob/master/SAxense.c),
licensed MPL-2.0) by Egor Vorontsov (Sdore). It reads raw unsigned-8-bit
two-channel PCM at 3000 Hz from stdin and continuously writes HID output
reports to a `/dev/hidrawN` device file. One report is written every 10.67 ms
(`SAMPLE_SIZE=64` samples at `SAMPLE_RATE=3000` Hz / 2 channels). The program
uses `CLOCK_MONOTONIC` POSIX timers and `mlockall` to keep itself in RAM.

Key facts verified from the source code:

- Report ID `0x32`, size 141 bytes, includes a CRC-32 using seed `0xA2`
- 64 PCM samples (stereo S8) are packed into a `0x12` sub-packet per report
- The timer interval is `1e9 * 64 / (3000 * 2) ≈ 10 667 000 ns ≈ 10.67 ms`
- No library dependencies beyond libc and POSIX timers
- Makefile consists of a single rule: `SAxense: SAxense.c` (implicit `cc`)
- License: Mozilla Public License 2.0

The tool works for both DualSense (USB PID `0CE6`) and DualSense Edge
(`0DF2`).

## Why SAxense is the right tool for Bluetooth haptics on Linux

`pydualsense` (the existing Python library that Dr.QP uses for input reading
via ROS `joy` node indirectly) **cannot trigger haptics over Bluetooth**.
Output reports on Linux's Bluetooth hidraw interface are not routed through the
same path as USB, and no Python library as of 2025 overcomes this limit. The
community consensus is that USB is required for haptics with `pydualsense`.

**SDL2 and SDL3** provide cross-platform haptic APIs
([SDL2 docs](https://wiki.libsdl.org/SDL2/CategoryHaptic),
[SDL3 docs](https://wiki.libsdl.org/SDL3/CategoryHaptic)) and are worth
considering for richer rumble effects. However, both go through the Linux
kernel's force-feedback (evdev FF) interface. On Linux Bluetooth, the
`hid-playstation` driver exposes only basic `FF_RUMBLE` through evdev, not the
raw HID output path needed for 3000 Hz stereo PCM haptics. SDL2 and SDL3 are
therefore suitable for the rumble MVP path (see
[#241](https://github.com/Dr-QP/Dr.QP/issues/241)) but do not replace SAxense
for audio-quality vibration waveforms over Bluetooth.

SAxense specifically addresses the PCM-haptics gap. It writes to the hidraw
device directly, bypassing the Linux input subsystem entirely. This works over
Bluetooth because it targets the Bluetooth HID output path (`uhid`).

The confirmed device glob used by SAxense is:

```sh
/sys/devices/virtual/misc/uhid/0005:054C:{0CE6,0DF2}.*/hidraw
```

This is distinct from the USB hidraw path and is only available when the
controller is connected over Bluetooth.

## Integration paths

Two usage patterns are documented in the SAxense README.

### Option A — ffmpeg with pre-recorded clips (recommended for Dr.QP)

```sh
dev="$(ls /sys/devices/virtual/misc/uhid/0005:054C:{0CE6,0DF2}.*/hidraw \
       | sed 's|^|/dev/|')"
ffmpeg -re -i /path/to/pattern.wav \
       -ac 2 -ar 3000 -f s8 - | ./SAxense > "$dev"
```

Produce three distinct WAV clips at 3000 Hz stereo (e.g., `mode_change.wav`,
`gait_change.wav`, `error.wav`) by authoring short pulse shapes in an audio
editor or synthesizing them with `sox` or `ffmpeg`. Each clip plays once per
invocation. This path needs only `ffmpeg` and standard POSIX tools; no
PipeWire involvement is required.

### Option B — PipeWire persistent sink

```sh
pw-cli -m load-module libpipewire-module-pipe-tunnel \
  tunnel.mode=sink pipe.filename=/dev/shm/SAxense.sock \
  audio.format=S8 audio.rate=3000 audio.channels=2 \
  node.name=SAxense \
  stream.props='{media.role=Haptics device.icon-name=input-gaming}'
./SAxense < /dev/shm/SAxense.sock > "$dev"
```

This keeps SAxense running continuously and lets any PipeWire-aware source
route to the haptics sink by name. More capable but requires a working PipeWire
session daemon, which is non-trivial on Ubuntu 24.04 / Raspberry Pi 5 — see
[known issues](#known-issues-with-pipewire-on-ubuntu-2404--raspberry-pi-5).

**For Dr.QP, Option A is the lower-risk integration start point.**

## Existing Dr.QP infrastructure fit

| Asset | Status | Location |
| --- | --- | --- |
| udev rules for DualSense hidraw (USB + BT) | ✅ Present | `docker/ros/ansible/playbooks/roles/dualsense_hidapi/tasks/main.yml` |
| `libhidapi-dev` installed | ✅ Present | same Ansible role |
| Bluetooth pairing documented | ✅ Present | `GettingStarted/ansible` |
| RPi 5 + Ubuntu 24.04 target documented | ✅ Present | `GettingStarted/setting-up-raspberry-pi` |
| Control-mode change decision point | ✅ Present | `drqp_brain/joystick_translator_node.py` |
| `ffmpeg` installed in runtime | ❌ Missing | Not in Ansible playbooks |
| SAxense build step | ❌ Missing | Not in Ansible playbooks |
| Process lifecycle for SAxense | ❌ Missing | No ROS 2 node or supervisor entry |
| Pre-recorded haptic WAV clips | ❌ Missing | No assets |
| Reconnect detection and re-spawn | ❌ Missing | No implementation |

The udev rules in `99-dualsense.rules` already set `MODE="0666"` for both USB
and Bluetooth hidraw paths — no permission changes are needed for SAxense.

## Estimated latency budget

SAxense itself introduces ~10.67 ms of buffering (64-sample write interval).
The remaining latency comes from the Bluetooth stack, not SAxense or hidraw:

| Stage | Estimated latency |
| --- | --- |
| SAxense PCM buffer | ~10–11 ms |
| Bluetooth HID output path | ~20–40 ms |
| Controller motor response | ~5–10 ms |
| Total (expected) | **~35–60 ms** |
| Total (degraded / high-load) | up to 200–500 ms |

The SAxense README explicitly warns that "up to a couple seconds in some rare
cases" is possible when using the PipeWire loopback capture mode. The ffmpeg
path avoids the loopback and is expected to be closer to the lower bound.

**These numbers are estimates from community sources and source code analysis.
They have not been measured on the Dr.QP target hardware.** The acceptance
criterion of 20 captured measurements must be met before any integration ships.

(known-issues-with-pipewire-on-ubuntu-2404--raspberry-pi-5)=
## Known issues with PipeWire on Ubuntu 24.04 / Raspberry Pi 5

Community documentation (tested July 2024 on RPi 5 + Ubuntu 24.04) confirms:

- The stock Ubuntu 24.04 `bluez` version (5.72 at the time) caused persistent
  Bluetooth audio disconnects or stutter after 60–90 seconds.
- The working fix requires building BlueZ ≥ 5.77 from source and setting
  `ControllerMode = dual` in `/etc/bluetooth/main.cfg`.
- PipeWire's default quantum can cause choppy audio under CPU load; the
  recommended workaround is `pw-metadata -n settings 0 clock.force-quantum 2048`.

For Option A (ffmpeg with pre-recorded clips), none of these issues apply
because SAxense reads from a pipe directly and does not involve the audio stack.

## Open issue in SAxense issue tracker

[Issue #1](https://github.com/egormanga/SAxense/issues/1) (opened July 2025,
still open) documents a correction to the HID packet structure: when the `unk`
bool in `packet_t` is `true`, the data length must be doubled. The current
source code does not set `unk = true` in its `packet_0x11` struct, so this
issue does not affect normal single-channel haptic output. However, it does
indicate that the HID format is not fully settled and the protocol may change.

## Acceptance criteria status

| Criterion | Status | Notes |
| --- | --- | --- |
| Reproducible setup procedure | Partial | udev rules and hidapi are in place. `ffmpeg`, SAxense build, and process management are not yet automated. |
| 3 distinct feedback patterns | Achievable | Pre-recorded 3000 Hz stereo WAV clips for mode\_change, gait\_change, and error can be authored with `sox` or `ffmpeg`. Not yet created. |
| Latency for 20 events | Not met | Hardware measurements required. Estimated 35–60 ms normal, up to 500 ms degraded. |
| Disconnect/reconnect behavior | Partial | Failure modes are known; implementation is unwritten. |
| CPU/memory impact | Not met | SAxense uses `mlockall` and a real-time timer. Impact on RPi 5 is unknown. |
| Clear recommendation | Met | Recommendation is Go with constraints. |
| Integration approach and fallback | Met | Detailed below. |

## Recommended integration approach

### Architecture

```
drqp_brain (ROS 2 node)
  └─ on mode/gait change event
       └─ HapticsBackend (optional, non-blocking)
            ├─ none (default, no-op)
            ├─ rumble_mvp (existing #241 path)
            └─ saxense (optional, if enabled and controller present)
                 └─ subprocess: ffmpeg -re -i <clip.wav> ... | SAxense > /dev/hidrawN
```

### Design constraints

1. **Decision point stays in `drqp_brain`** — the existing
   `joystick_translator_node.py` already holds the mode-change logic.
2. **Non-blocking invocation** — launch a two-process pipeline with
   `subprocess.Popen`: start `ffmpeg` with `stdout=PIPE`, start `SAxense`
   with `stdin` connected to that pipe, and do not wait for completion. The
   haptic clip plays asynchronously.
3. **Optional at runtime** — if `SAxense` binary or hidraw device is absent,
   log a one-time warning and continue without haptics.
4. **Graceful degradation** — if the subprocess fails (exit code non-zero,
   device disappears), log the error and disable the backend until the next
   controller reconnect event.
5. **No new ROS topics** — feedback dispatch is an implementation detail of
   the brain node, not a published interface.

### Build additions needed

- Ansible: install `ffmpeg` (`apt`)
- Ansible or Dockerfile: `git clone https://github.com/egormanga/SAxense`, run
  `make`, install binary to `/usr/local/bin/SAxense`
- Asset directory: store pre-recorded haptic clips (mode\_change.wav,
  gait\_change.wav, error.wav) as project resources

### Process lifecycle

- SAxense is launched per-event as a short-lived subprocess (`ffmpeg | SAxense`)
- On controller disconnect, the subprocess exits naturally (write to hidraw fails)
- On reconnect, the next event launch re-discovers the hidraw device path

## Failure modes and mitigations

| Failure | Mitigation |
| --- | --- |
| hidraw device absent at startup | Log warning; skip haptics silently |
| SAxense binary absent | Log warning on first invocation only; degrade to no haptics |
| subprocess crash mid-clip | Launch asynchronously, capture stderr, detect failure via child return code or polling; log once; continue |
| Multiple rapid events | Allow concurrent clips or cancel previous; do not block control loop |
| hidraw path changes after reconnect | Re-discover device path on each invocation |
| PipeWire latency spike (Option B) | Use Option A (ffmpeg) instead |

## What must happen before code ships

1. Measure actual trigger-to-output latency on target hardware (≥20 events)
2. Confirm haptic patterns are perceptibly distinct at the expected latency
3. Test reconnect: disconnect controller mid-run, reconnect, verify next event works
4. Measure CPU and memory overhead of `ffmpeg | SAxense` subprocess pair
5. Verify `mlockall` succeeds under the Dr.QP process environment

## Recommendation

**Go with constraints.**

SAxense is the correct tool for DualSense Bluetooth haptics on Linux and
integrates well with Dr.QP's existing infrastructure. The core integration
surface is small (one subprocess per event) and the existing udev rules already
grant the needed permissions.

The constraints are:

- New project (two commits, no stable release); protocol may still change
- Target-hardware measurements not yet in hand
- Build and asset provisioning work required in Ansible

**The recommended next action is to run the hardware validation spike on a
real Raspberry Pi 5 + DualSense over Bluetooth, using the measurement checklist
above, and then open a follow-up implementation issue linked to
[#241](https://github.com/Dr-QP/Dr.QP/issues/241).**

## Alternatives considered and rejected

| Alternative | Why rejected |
| --- | --- |
| `pydualsense` for haptics | Cannot send haptic output over Bluetooth (USB-only output) |
| `dualsensectl` | Configuration tool; no haptic audio streaming capability |
| `dualsense-controller` PyPI package | Same USB-only haptics limitation |
| SDL2 haptic API ([docs](https://wiki.libsdl.org/SDL2/CategoryHaptic)) | Cross-platform rumble and force-feedback via Linux kernel evdev FF interface; Python binding via `pysdl2`; supports `FF_RUMBLE` and waveform effects where the kernel exposes them; on Linux Bluetooth, `hid-playstation` driver exposes only basic `FF_RUMBLE` — no raw-HID PCM path; rumble use case already covered by #241 |
| SDL3 haptic API ([docs](https://wiki.libsdl.org/SDL3/CategoryHaptic)) | Updated API with improved DualSense support including `SDL_HAPTIC_CUSTOM` for custom waveforms on supporting hardware; still kernel-FF-based on Linux Bluetooth; does not expose the 3000 Hz stereo S8 PCM path needed for audio-quality haptics; Python bindings (`pysdl3` / ctypes) less mature; viable for richer rumble or adaptive trigger effects but does not replace SAxense for PCM haptics |
| PipeWire loopback (SAxense Option B) | Higher latency risk, requires working BT audio stack; deferred |
| Kernel rumble driver (`FF_RUMBLE`) | Basic rumble only; no frequency-shaped vibration; already covered by #241 |
