---
type: codebase
description: Packages used only off the robot — Gazebo simulation and its launch tests, the GUI keyboard controller, and the robot MCP server.
source: packages/simulation
source_digest: sha256:19be80cc2f00edc7194f4bbaa06e462283d1645579315168a26e969969397dc3
verified:
  by: claude-code/opus-5.5
  at: 2026-10-12T22:32:35Z
stale_after: 2027-01-10
generated:
  by: claude-code/opus-5.5
  at: 2026-10-12T22:32:35Z
sources:
- id: code
  resource: packages/simulation
---

# packages/simulation

The colcon packages built in the devcontainer and CI but not in the deploy
image. They drive the same `drqp_brain` bringup against Gazebo (gz-sim) in place
of the servos, or they act as extra input sources on the same topics.

## Contains

[drqp_gazebo](simulation/drqp_gazebo.md)

[drqp_keyboard_control](simulation/drqp_keyboard_control.md)

[drqp_robot_mcp](simulation/drqp_robot_mcp.md)

## Depends on

- [packages/runtime](runtime.md): all three packages depend on `drqp_brain` or
  its topics.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/simulation/drqp_gazebo/package.xml` — `depend drqp_brain`
- `packages/simulation/drqp_keyboard_control/package.xml` —
  `exec_depend drqp_brain`
