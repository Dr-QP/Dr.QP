---
type: codebase
description: Packages used only off the robot — Gazebo simulation and its launch tests, the GUI keyboard controller, and the robot MCP server.
source: packages/simulation
source_digest: sha256:b34767b7f48c537ca4d768d33d94fbb5b3f5791fec5903e87f6581f5d7471315
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
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
