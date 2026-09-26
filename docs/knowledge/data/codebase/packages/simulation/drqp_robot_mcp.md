---
type: codebase
description: FastMCP stdio server letting AI agents start/stop the Gazebo simulation, boot and drive the robot, record state, and read world poses.
source: packages/simulation/drqp_robot_mcp
source_digest: sha256:1397da1804265175c6ceb44cbda353e45188a22b61157175f5e4f7022e2c0b7a
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: packages/simulation/drqp_robot_mcp
---

# drqp_robot_mcp

"The canonical workspace implementation of the Dr.QP MCP server." It is both a
ROS package and a `uv` path dependency, installed editable into `.venv`.
`.vscode/mcp.json` registers it as `drqp_robot` and runs
`.venv/bin/drqp_robot_mcp`.

## Public surface

The tools and resources are described in
[api-robot-mcp](../../api-robot-mcp.md). Code entry points:

- `server.main`: runs `mcp.run()` over stdio.
- `RobotMcpController`: lifecycle orchestration.
- `runtime.RosRuntimeSession`: the in-process rclpy session.

## How it works

- **Simulation lifecycle:** `simulation.start` runs
  `ros2 launch drqp_gazebo sim.launch.py sim_gui:=<gui>` detached. It writes a
  PID file and a log under the runtime directory (`ROS_HOME` when set).
- **Robot lifecycle:** `robot.boot` and `robot.shutdown` publish lifecycle
  events on `/robot_event` and wait on `/robot_state` for `torque_on` or
  `finalized`.
- **Motion:** motion tools publish `MovementCommand` on
  `/robot/movement_command`.
- **World state:** comes from Gazebo transport.

## Depends on

- `mcp` (FastMCP), `rclpy`, `drqp_interfaces`, gz-transport Python bindings
- [drqp_gazebo](drqp_gazebo.md), launched as a subprocess

## Invariants & gotchas

- `py_packages/drqp_robot_mcp/` is a separate, older copy of the server with no
  manifest (last touched in #368). It is not the one installed. `pyproject.toml`
  points `drqp_robot_mcp` at this package.
- The server sets `GZ_PARTITION` the same way as `sim.launch.py`
  (`_ensure_gazebo_partition_environment`), so it can see its own simulation.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/simulation/drqp_robot_mcp/drqp_robot_mcp/server.py:11` — `FastMCP`
  instance
- `packages/simulation/drqp_robot_mcp/drqp_robot_mcp/server.py:289` — `main`
- `packages/simulation/drqp_robot_mcp/drqp_robot_mcp/controller.py:39` —
  `RobotMcpController`
- `packages/simulation/drqp_robot_mcp/drqp_robot_mcp/controller.py:64` —
  `boot_up`
- `packages/simulation/drqp_robot_mcp/drqp_robot_mcp/runtime.py:56` —
  `RosRuntimeSession`
- `packages/simulation/drqp_robot_mcp/drqp_robot_mcp/runtime.py:645` —
  `start_simulation`
- `packages/simulation/drqp_robot_mcp/drqp_robot_mcp/runtime.py:877` —
  `get_runtime_directory`
- `pyproject.toml:70` — `uv` path source
