---
type: bug
description: bringup.launch.py declares the load_joystick launch argument twice with different descriptions.
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T12:00:00Z
sources:
- id: bringup
  resource: packages/runtime/drqp_brain/launch/bringup.launch.py
---

# Bug: bringup declares load_joystick twice

## Symptom

`ros2 launch drqp_brain bringup.launch.py --show-args` can list `load_joystick`
with either description ("Load joy game_controller_node" or "Load drqp_joy
game_controller_node"). Its default could then drift silently if the two
declarations diverge.

## Reproduction

Read `bringup.launch.py`. `DeclareLaunchArgument(name='load_joystick', ...)`
appears at the top level and again inside the `GroupAction`.

## Root cause

The second declaration is left over inside the group. Both default to `false`,
so today's behavior is unaffected.

## Fix

Delete the declaration inside the group. Keep the top-level one, which the
`LaunchConfiguration` reads.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_brain/launch/bringup.launch.py:68` — top-level
  declaration
- `packages/runtime/drqp_brain/launch/bringup.launch.py:112` — duplicate inside
  the group
