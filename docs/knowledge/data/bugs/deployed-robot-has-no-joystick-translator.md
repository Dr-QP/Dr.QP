---
type: bug
description: The deploy image's default bringup leaves load_joystick false, so no node on the robot turns /joy into /robot/movement_command or kill-switch events.
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T12:00:00Z
sources:
- id: deploy
  resource: docker/ros/deploy/ros-deploy.Dockerfile
- id: bringup
  resource: packages/runtime/drqp_brain/launch/bringup.launch.py
- id: ansible
  resource: docker/ros/ansible/playbooks/100_startup_service.yml
- id: map
  resource: docs/knowledge/data/codebase/docker/ros.md
---

# Bug: deployed robot has no joystick translator

## Symptom

This is read from the code; it has not been observed on hardware. When the robot
runs the Ansible-installed services, nothing translates DualSense input:

- The joy node publishes `/joy`.
- Nothing publishes `/robot/movement_command`.
- Nothing publishes the PS or touchpad `kill_switch_pressed` event.

The robot would not respond to the gamepad.

## Reproduction

1. Provision the Pi with `100_startup_service.yml`, or run
   `docker run ghcr.io/dr-qp/jazzy-ros-deploy:edge` with no command.
2. Run the joystick service, which runs
   `ros2 run drqp_joy game_controller_node`.
3. `ros2 node list` shows no `drqp_joystick_translator`.

## Root cause

The two pieces are split in a way that leaves the translator out:

- The deploy image's `CMD` is `ros2 launch drqp_brain bringup.launch.py` with no
  arguments.
- `bringup.launch.py` gates both `game_controller_node` and
  `drqp_joystick_translator` on `load_joystick`, which defaults to `false`.
- The separate joystick container, split out so that the gamepad can reconnect
  (see
  [Joystick in its own container](../architecture/joystick-container-split.md)),
  runs only `game_controller_node`.

So the translator falls between the two containers.

*Unknown:* whether the Pi is currently started some other way that passes
`load_joystick:=true`. Nothing in the repo does.

## Fix

Not decided. The options:

- **Split the launch argument.** Separate `load_translator` from `load_joystick`
  in bringup, default it to `true`, and keep `game_controller_node` in its own
  container.
- **Pass an argument from the control service.** Have the control service pass a
  translator-only argument.
- **Move the translator into the joystick container.** Run it next to
  `game_controller_node`. This couples it to reconnect restarts.

The fix should land with a launch test that asserts which nodes the deploy
command starts.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `docker/ros/deploy/ros-deploy.Dockerfile:87` — default `CMD` without arguments
- `packages/runtime/drqp_brain/launch/bringup.launch.py:68` — `load_joystick`
  default `false`
- `packages/runtime/drqp_brain/launch/bringup.launch.py:127` — translator gated
  on `load_joystick`
- `docker/ros/ansible/playbooks/100_startup_service.yml:79` — control container
  runs the image default
- `docker/ros/ansible/playbooks/100_startup_service.yml:124` — joystick
  container runs only `game_controller_node`
