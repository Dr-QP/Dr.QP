---
type: architecture
description: Why gait phase advances by measured wall time with per-gait cycle times in seconds, decoupled from the control loop rate.
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T12:00:00Z
sources:
- id: walk
  resource: packages/runtime/drqp_brain/drqp_brain/walk_controller.py
- id: brain
  resource: packages/runtime/drqp_brain/drqp_brain/brain_node.py
- id: specs
  resource: docs/agents/specs/locomotion/README.md
- id: commit
  resource: git 60dfda1 "Make gait targets pure and phase advancement time-based (locomotion spec 02) (#434)", 2026-07-11
---

# Time-based gait timing

## Decision

- **Wall-time phase.** Gait phase advances once per control tick by the measured
  `dt`, clamped to `[0, 2/rate]`: `WalkController.advance(dt, …)` is the only
  state transition.
- **Pure targets.** Foot targets come from `targets_at(phase, steering)`, which
  has no side effects.
- **Gait speed in seconds.** Speed is `cycle_time_sec` per gait: tripod 1.25 s,
  ripple 1.5625 s, wave 2.5 s.
- **Rate-independent smoothing.** Steering is smoothed with
  `alpha = 1 − exp(−dt/τ)`, where τ = 0.35 s.

## Why

Locomotion spec 02 (#434) decoupled behavior from the tick rate, so that
`control_rate_hz` (spec 08, default 25 Hz, range 5–100) can change without
changing how fast the robot walks. Pure targets let the brain fill a 2-point
trajectory window without mutating state. On an IK failure or an unchanged tick,
the loop can then simply not publish. The cycle times and τ were chosen to
preserve the observed speed and feel of the old 8 Hz "double-advance" loop, and
tests assert them.

## Rejected alternatives

- **Frame-based `phase_steps_per_cycle`.** It coupled gait speed to fps and was
  removed in #434.
- **Snapshot and restore of motion state around speculative solves**
  (`_snapshot_motion_state` / `_restore_motion_state`). Deleted in #434 in favor
  of pure evaluation.

## Consequences

- Retuning a gait means editing `cycle_time_sec` in `brain_node.py`, not the
  rate.
- A stalled tick longer than 2 control periods is clamped, so the phase does not
  jump after a hiccup.
