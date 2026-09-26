---
type: feature
stage: proposed
status: draft
description: Have the joystick translator drive rich SDL haptic effects through joy/set_haptic, which drqp_joy already serves but nothing publishes to.
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T12:00:00Z
sources:
- id: joy
  resource: packages/runtime/drqp_joy/src/game_controller.cpp
- id: msg
  resource: packages/runtime/drqp_interfaces/msg/HapticEffect.msg
- id: translator
  resource: packages/runtime/drqp_brain/drqp_brain/joystick_translator_node.py
---

# Translator haptic effects

## Purpose

`drqp_joy` subscribes to `joy/set_haptic` (`drqp_interfaces/HapticEffect`) and
can play constant, periodic, and ramp effects. Nothing in the repo publishes to
it. The translator sends only rumble pulses on `/joy/set_feedback` (gait and
control-mode patterns in `haptics.py`). This feature would use the richer
channel to report robot state through the controller.

## Behaviour

Proposed; nothing below is decided:

- On lifecycle changes (`torque_on`, `torque_off`, `servos_rebooting`), the
  translator publishes a distinct `HapticEffect` so the operator can feel the
  state without looking at the robot.
- Kinematic clamping or balance saturation (brain events) could map to a short
  warning effect.

## Edge cases

- Controllers without full haptic support: `drqp_joy` must degrade to rumble or
  a no-op.
- A kill-switch press must never be delayed by effect playback.

## Open questions

- Which states and events deserve feedback, and whether the brain or the
  translator owns the mapping.
- Whether the rumble patterns in `haptics.py` should move to `HapticEffect`
  entirely.
