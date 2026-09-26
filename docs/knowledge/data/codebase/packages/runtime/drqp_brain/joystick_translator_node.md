---
type: codebase
description: The drqp_joystick_translator node and its helpers — maps DualSense Joy messages to MovementCommand, lifecycle events and balance toggles, and schedules rumble feedback.
source:
- packages/runtime/drqp_brain/drqp_brain/joystick_translator_node.py
- packages/runtime/drqp_brain/drqp_brain/joystick_input_handler.py
- packages/runtime/drqp_brain/drqp_brain/joystick_button.py
- packages/runtime/drqp_brain/drqp_brain/haptics.py
source_digest: sha256:c3151c3ea0cc3246eb5ac8afc6702036ce17c151862703ee772eab8adc5ba92c
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
sources:
- id: code
  resource: packages/runtime/drqp_brain/drqp_brain/joystick_translator_node.py
---

# joystick_translator_node

The gamepad-specific layer between `drqp_joy` and the semantic topics. It is the
only code that knows DualSense button and axis indices.

## Public surface

- The `drqp_joystick_translator` executable (`JoystickTranslatorNode`).

- Subscribes to `/joy`.

- Publishes:

  - `/robot/movement_command`
  - `/robot_event`
  - `/robot/balance_mode` (latched)
  - `/joy/set_feedback`

- Button bindings:

  | Button           | Action                |
  | ---------------- | --------------------- |
  | D-pad left/right | previous/next gait    |
  | L1               | next control mode     |
  | R1               | toggle balance        |
  | PS, touchpad     | `kill_switch_pressed` |
  | Start            | `reboot_servos`       |
  | Select           | `finalize`            |

- Helper classes:

  - `JoystickInputHandler` and `ControlMode` (Walk, BodyPosition, BodyRotation).
  - `JoystickButton`: press-edge detection, emitting `Tapped` on a
    released→pressed transition.
  - `ButtonIndex` and `ButtonAxis`.
  - `HapticFeedbackScheduler`, with `gait_feedback_pattern` and
    `control_mode_feedback_pattern`.

## How it works

Every `Joy` message:

1. Updates the buttons, which fire their callbacks.
2. Maps the axes according to the control mode:
   - Walk: left stick → stride x/y, left trigger → stride z, right x → rotation.
   - BodyPosition and BodyRotation: the sticks set the body pose.
3. Publishes one `MovementCommand` carrying the current gait name.

Gait and mode changes schedule rumble pulses: left channel for gait, right for
mode, with a cooldown and de-duplication. A 20 ms timer dispatches the pulses as
`JoyFeedback`.

## Depends on

- [drqp_joy](../drqp_joy.md): the `Joy` layout
- [drqp_interfaces](../drqp_interfaces.md)
- [drqp_kinematics](../drqp_kinematics.md): `Point3D`

## Invariants & gotchas

- `ButtonIndex` values follow SDL3's gamepad button order, because `drqp_joy`
  fills `Joy.buttons` by the SDL button enum. For example, PS is 5 and the
  touchpad is 20. A different joy driver would remap them silently.
- The left trigger rests at −1 on some platforms and 0 on others. The code
  interpolates the range [−1, 0] onto [1, 0].
- The balance toggle is a local flag and can drift from the brain's actual state
  (see
  [Bug: balance toggle desyncs from the brain](../../../../bugs/balance-toggle-desyncs-from-brain.md)).
- On the deployed robot, this node is not launched (see
  [Bug: deployed robot has no joystick translator](../../../../bugs/deployed-robot-has-no-joystick-translator.md)).

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_brain/drqp_brain/joystick_translator_node.py:44` —
  `JoystickTranslatorNode`
- `packages/runtime/drqp_brain/drqp_brain/joystick_translator_node.py:69` —
  button bindings
- `packages/runtime/drqp_brain/drqp_brain/joystick_translator_node.py:106` —
  feedback dispatch timer
- `packages/runtime/drqp_brain/drqp_brain/joystick_translator_node.py:128` —
  `_publish_movement_command`
- `packages/runtime/drqp_brain/drqp_brain/joystick_input_handler.py:95` — axis
  mapping per control mode
- `packages/runtime/drqp_brain/drqp_brain/joystick_button.py:35` — `ButtonIndex`
- `packages/runtime/drqp_brain/drqp_brain/joystick_button.py:84` — press-edge
  detection
- `packages/runtime/drqp_brain/drqp_brain/haptics.py:66` —
  `HapticFeedbackScheduler`
