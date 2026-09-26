---
type: codebase
description: Pygame GUI with keyboard and virtual sticks that publishes MovementCommand, lifecycle events and balance toggles for the simulated robot.
source: packages/simulation/drqp_keyboard_control
source_digest: sha256:1cfec22c7fe4dc765b3939057fc68fea7f873fc12ffc04d9bf530cc01d9abadd
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: packages/simulation/drqp_keyboard_control
---

# drqp_keyboard_control

"GUI keyboard and virtual stick controller for Dr.QP simulation." It is a second
input source that stands in for the DualSense and publishes the same semantic
topics as the joystick translator.

## Public surface

- The `drqp_keyboard_control` executable (`KeyboardControlNode`).
- Publishes:
  - `/robot/movement_command` (`MovementCommand`), on a timer.
  - `/robot_event` (`String`).
  - `/robot/balance_mode` (`Bool`, latched).

## How it works

The modules are layered so that only one of them touches Pygame pixels:

- `control_state`: the input model, with virtual sticks and axes.
- `gui_controls` and `layout`: Pygame-free widget models and a flexbox-style
  layout.
- `ui`: the declarative widget tree.
- `keymap`: translates Pygame key codes.
- `renderer`: the only drawing module.
- `keyboard_control_app`: the composition root.

The node turns the resulting state into commands.

## Depends on

- [drqp_interfaces](../runtime/drqp_interfaces.md), `rclpy`, `python3-pygame`

## Invariants & gotchas

- Rendering is isolated in `renderer.py` so that the models and layout are
  testable without a display. The tests in `test/` rely on this.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/simulation/drqp_keyboard_control/drqp_keyboard_control/keyboard_control_node.py:44`
  — `KeyboardControlNode`
- `packages/simulation/drqp_keyboard_control/drqp_keyboard_control/keyboard_control_node.py:62`
  — movement publisher
- `packages/simulation/drqp_keyboard_control/drqp_keyboard_control/keyboard_control_node.py:77`
  — publish timer
- `packages/simulation/drqp_keyboard_control/drqp_keyboard_control/keyboard_control_app.py:31`
  — composition root
- `packages/simulation/drqp_keyboard_control/drqp_keyboard_control/control_state.py:106`
  — `GuiControlState`
- `packages/simulation/drqp_keyboard_control/drqp_keyboard_control/renderer.py:43`
  — `PygameRenderer`
