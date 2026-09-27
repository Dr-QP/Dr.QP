---
type: bug
stage: done
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
- id: test
  resource: packages/runtime/drqp_brain/test/test_bringup_launch_nodes.py
- id: map
  resource: docs/knowledge/data/codebase/docker/ros.md
---

# Bug: deployed robot has no joystick translator

## Symptom

When the robot runs the Ansible-installed services, nothing translates DualSense
input:

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

So the translator falls between the two containers. The maintainer confirmed
that the repo is everything the deployment runs, so no other path starts it.

The gap dates from #253 (`14cc28a`). Before it, `brain_node` subscribed to
`/joy` itself. #253 moved that into the translator but gated the translator on
`load_joystick`, which the robot already left off because the gamepad node ran
in its own container.

## Fix

Bringup has a separate `load_joystick_translator` argument, default `true`.
`load_joystick` now gates only `game_controller_node`, which stays in its own
container. The translator runs in the control container next to the brain. It
needs no device access and publishes only when `/joy` arrives, so it also runs
harmlessly in simulation and in the MoveIt demo.

`test_bringup_launch_nodes.py` evaluates the launch conditions with the default
arguments and asserts that the translator starts and the gamepad node does not.

## Key references

Verified anchor points (line numbers as of 2026-09-27):

- `docker/ros/deploy/ros-deploy.Dockerfile:87` — default `CMD` without arguments
- `packages/runtime/drqp_brain/launch/bringup.launch.py:78` —
  `load_joystick_translator` default `true`
- `packages/runtime/drqp_brain/launch/bringup.launch.py:131` — translator gated
  on `load_joystick_translator`
- `packages/runtime/drqp_brain/test/test_bringup_launch_nodes.py:77` — default
  bringup starts the translator
- `docker/ros/ansible/playbooks/100_startup_service.yml:79` — control container
  runs the image default
- `docker/ros/ansible/playbooks/100_startup_service.yml:124` — joystick
  container runs only `game_controller_node`
