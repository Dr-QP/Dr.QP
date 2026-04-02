# SAxense feasibility for DualSense audio and haptics

This spike is a desk-research assessment of SAxense as a possible optional
backend for DualSense feedback in Dr.QP. It does not claim target-hardware
validation because the sandbox used for this issue does not provide a paired
DualSense controller or Raspberry Pi runtime.

## Executive summary

- **Decision:** No-go for project adoption at this time
- **Why:** SAxense appears useful for experimental DualSense **haptics over
  Bluetooth**, but it does not provide a validated path for general controller
  audio output, and the project environment still lacks target-device
  measurements for latency, reconnect behavior, and operational stability.
- **Fallback:** Keep the rumble-only MVP tracked by
  [#241](https://github.com/Dr-QP/Dr.QP/issues/241) as the supported baseline.

## What SAxense appears to provide

SAxense is an external proof-of-concept tool that streams data to the DualSense
hidraw interface to drive the controller haptics over Bluetooth. Its published
examples use shell pipelines together with `ffmpeg` or PipeWire-style audio
plumbing rather than a stable application API.

Based on public project documentation, SAxense is best understood as:

- a **Bluetooth haptics experiment**
- driven by **raw HID output**
- usually fed by **PipeWire** or `ffmpeg`
- not a turnkey Linux backend for general DualSense speaker or headset audio

That distinction matters for this issue: the title asks about audio output to
DualSense, while the available evidence mostly supports experimental haptics.

## Existing Dr.QP environment fit

The current repository already contains useful DualSense prerequisites:

- Raspberry Pi guidance currently targets Ubuntu 24.04 on Raspberry Pi 5 in
  {doc}`../GettingStarted/setting-up-raspberry-pi`
- Ansible documents controller pairing in
  {doc}`../GettingStarted/ansible`
- `docker/ros/ansible/playbooks/roles/dualsense_hidapi/tasks/main.yml` already
  installs `libhidapi-dev` and configures udev rules for DualSense hidraw
  access over USB and Bluetooth
- `packages/runtime/drqp_brain/drqp_brain/joystick_translator_node.py` is the
  current single source of truth for gait and control-mode changes that would
  trigger operator feedback

However, the repository does **not** currently automate or document:

- PipeWire setup for a dedicated haptics sink
- `ffmpeg` installation in runtime provisioning
- lifecycle management for an external SAxense process
- reconnect logic for a backend that disappears while the control node keeps
  running

## Acceptance criteria status

Because this spike was limited to repository inspection plus external research,
the issue acceptance criteria are only partially answerable:

| Criterion | Status | Notes |
| --- | --- | --- |
| Reproducible setup procedure | Partial | Existing repo setup covers pairing, udev rules, and hidapi. SAxense-specific PipeWire and process supervision are still missing. |
| 3 distinct feedback patterns | Unverified | Likely feasible in principle, but not validated on target hardware in this spike. |
| Latency for 20 events | Not met | No target-device measurements were captured. |
| Disconnect/reconnect behavior | Partial | Failure modes are identifiable, but not empirically validated. |
| CPU/memory impact | Not met | No target-device measurements were captured. |
| Clear recommendation | Met | Recommendation is no-go for adoption now. |
| Integration approach and fallback | Met | A constrained integration sketch is included below. |

## Key risks

### 1. Feature mismatch

The strongest published SAxense use case is DualSense haptics, not general audio
output to the controller. That makes it a weak match for any design that relies
on speaker/headset audio as the primary enhancement.

### 2. Operational complexity

SAxense depends on raw hidraw access plus external media plumbing. For Dr.QP,
that means introducing:

- process supervision
- startup ordering against Bluetooth pairing
- conflict handling if another process opens the same controller
- graceful degradation when the backend is missing

### 3. Missing target evidence

Issue #241 asks for feedback that is immediate, reliable, and non-blocking.
Those properties depend on measured end-to-end latency and failure recovery on
the actual Raspberry Pi + Bluetooth stack. This spike did not produce that
evidence.

## Failure modes to expect

Even without direct hardware tests, the likely operational failure modes are
clear:

- controller unavailable at startup
- controller reconnects after SAxense starts and the hidraw path changes
- SAxense exits while `drqp_brain` keeps running
- multiple consumers compete for the same hidraw device
- PipeWire or `ffmpeg` introduces extra buffering or startup delay
- firmware or kernel changes alter behavior across controllers

For Dr.QP, every one of these cases would require a warning log and fallback to
the existing non-audio path instead of blocking the control loop.

## Recommended architecture if revisited later

If SAxense is revisited after target-hardware testing, keep it behind a strict
optional boundary:

1. **Decision point stays in `drqp_brain`** at the same place mode changes are
   finalized today.
2. **Dispatch must be non-blocking**, for example by publishing feedback intents
   to a dedicated worker or sidecar process.
3. **Backend contract must be optional**:
   - `none`
   - rumble-only MVP
   - experimental `saxense`
4. **Failure behavior must degrade cleanly**:
   - log once with the backend state and reason
   - keep control operation alive
   - fall back to rumble-only or terminal logging

That keeps issue #241's rumble mapping as the supported baseline while leaving a
place for future experiments.

## Recommendation

**No-go for immediate adoption.**

SAxense is interesting as a lab experiment for optional Bluetooth haptics, but
it is not yet a good project dependency for Dr.QP because:

- the evidence found is stronger for haptics than controller audio output
- the integration surface is process-heavy and operationally fragile
- this spike did not produce the target Raspberry Pi measurements required to
  clear issue #241's latency and reliability expectations

## Suggested follow-up if the team wants a second spike

Only reopen SAxense evaluation if all of the following are available:

1. a real Raspberry Pi target with the intended Ubuntu, kernel, and Bluetooth
   stack
2. a paired DualSense controller
3. a scripted measurement harness that records:
   - at least 20 trigger-to-output events
   - reconnect timing
   - CPU and memory usage during burst feedback
4. a supervised process model proving that backend failure does not interrupt
   control flow

Until then, issue [#241](https://github.com/Dr-QP/Dr.QP/issues/241) should
continue on the rumble-only path.
