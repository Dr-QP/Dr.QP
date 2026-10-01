---
type: bug
description: The joystick translator and keyboard GUI flip a local balance flag on each press, but the brain can refuse or drop balance mode without telling them, so the next press sends the wrong value.
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
sources:
- id: translator
  resource: packages/runtime/drqp_brain/drqp_brain/joystick_translator_node.py
- id: gui
  resource: packages/simulation/drqp_keyboard_control/drqp_keyboard_control/keyboard_control_node.py
- id: brain
  resource: packages/runtime/drqp_brain/drqp_brain/brain_node.py
---

# Bug: balance toggle desyncs from the brain

## Symptom

This is read from the code; it has not been observed on hardware. After the
brain refuses or ends balance mode, the operator must press R1 (or the GUI
toggle) twice to turn it on again. The first press sends `False`, which is a
no-op.

## Reproduction

1. With the robot in `torque_off`, press R1. The translator sets its flag to
   `True` and publishes `True`.
2. `HexapodBrain.process_balance_mode` refuses, because the state is not
   `torque_on`, and stays off.
3. Stand the robot up (`torque_on`) and press R1. The translator flips to
   `False` and publishes `False`, so balance stays off.

The same happens after the brain drops balance on its own: when the IMU goes
stale or unavailable, or when the robot leaves `torque_on`.

## Root cause

- Both input sources keep `balance_mode_enabled` locally and publish its
  negation on each press, on the latched `/robot/balance_mode`.
- The brain treats the topic as a request, which it may reject
  (`enable_imu_balance` off, not `torque_on`, no fresh IMU tilt), and it can
  disable balance later (`_disable_balance_mode`).
- The brain publishes no accepted-state feedback, so the sources never resync.

A related hazard: the translator and the GUI are both latched publishers on the
same topic, and each publishes `False` at startup. A late-joining brain receives
both samples in an order it cannot control.

## Fix

Not decided. The options:

- **Publish the actual state.** The brain publishes the balance state it
  actually applies, latched (for example `/robot/balance_mode_active`). The
  inputs toggle from that value.
- **Make it an event.** Turn the request into a stateless event such as
  `toggle_balance`, and let the brain own the state.

Either fix needs a node test that covers the refuse-then-press sequence.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_brain/drqp_brain/joystick_translator_node.py:180` —
  translator local toggle
- `packages/simulation/drqp_keyboard_control/drqp_keyboard_control/keyboard_control_node.py:95`
  — GUI local toggle
- `packages/runtime/drqp_brain/drqp_brain/brain_node.py:421` —
  `process_balance_mode` may refuse
- `packages/runtime/drqp_brain/drqp_brain/brain_node.py:443` —
  `_disable_balance_mode` without feedback
