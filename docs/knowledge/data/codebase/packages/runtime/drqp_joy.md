---
type: codebase
description: C++ SDL3 game-controller node publishing sensor_msgs/Joy and playing rumble and haptic effects on the DualSense.
source: packages/runtime/drqp_joy
source_digest: sha256:0a7a24fa0b95b0c94dc87bfd445b73d5fdbf27d72988ef95491fd6cf0c085f14
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: packages/runtime/drqp_joy
---

# drqp_joy

"Dr.QP game controller node — SDL3 port with full dual-motor rumble and haptic
support." It is an `rclcpp` component (`drqp_joy::GameController`), also
installed as the `game_controller_node` executable. It gets SDL3 through
`sdl3_vendor`, which is built as a console-only SDL with no X11 or Wayland.

## Public surface

- Publishes `joy` (`sensor_msgs/Joy`). The buttons and axes are sized to SDL's
  gamepad button and axis counts.
- Subscribes to:
  - `joy/set_feedback` (`sensor_msgs/JoyFeedback`): simple rumble.
  - `joy/set_haptic` (`drqp_interfaces/HapticEffect`): full SDL haptic effects.
- Parameters:
  - `device_id` and `device_name`
  - `deadzone` (0.05)
  - `autorepeat_rate` (20.0)
  - `sticky_buttons`
  - `coalesce_interval_ms`
  - `feedback_rumble_duration_ms`

## How it works

The node pumps SDL events from a ROS wall timer on the executor thread. The poll
period comes from the autorepeat and coalesce settings. It discovers gamepads,
handles hotplug, and maps SDL gamepad state onto the `Joy` message.

## Depends on

- `sdl3_vendor` (in `packages/vendor`)
- [drqp_interfaces](drqp_interfaces.md)
- `rclcpp_components`, `sensor_msgs`

## Invariants & gotchas

- SDL3 requires `SDL_PumpEvents` and `SDL_WaitEvent*` on the main thread. That
  is why polling runs on the executor timer rather than a worker thread.
- On the robot, this node runs in its own container, restarted on input-device
  changes, because Docker cannot hot-plug `/dev/input/event*` (see
  [docker/ros](../../docker/ros.md)).

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_joy/include/drqp_joy/game_controller.hpp:59` —
  `GameController`
- `packages/runtime/drqp_joy/src/game_controller.cpp:188` — `joy` publisher
- `packages/runtime/drqp_joy/src/game_controller.cpp:202` — `joy/set_haptic`
  subscription
- `packages/runtime/drqp_joy/src/game_controller.cpp:218` — SDL event poll timer
- `packages/runtime/drqp_joy/src/game_controller.cpp:751` — component
  registration
- `packages/runtime/drqp_joy/README.md` — SDL vendoring and main-thread
  rationale
