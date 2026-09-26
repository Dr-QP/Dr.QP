---
type: codebase
description: The MCP tools and resources the drqp_robot_mcp server exposes to AI agents over stdio.
source: packages/simulation/drqp_robot_mcp/drqp_robot_mcp/server.py
source_digest: sha256:10547f6f81ea8ed32f3a4df4df6bea875f86393226d34f13b222403c3b088fa8
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: packages/simulation/drqp_robot_mcp/drqp_robot_mcp/server.py
---

# api-robot-mcp

The FastMCP server `Dr.QP Robot MCP`, run over stdio by the `drqp_robot_mcp`
executable. It is implemented by
[drqp_robot_mcp](packages/simulation/drqp_robot_mcp.md).

## Tools

- `simulation.start` and `simulation.stop`: launch or stop
  `drqp_gazebo sim.launch.py` (timeout 120 s).
- `simulation.state`, `simulation.robot_state`, and `simulation.world_state`:
  snapshots of the simulator, the simulated robot, and the Gazebo world poses.
- `robot.boot`: drives the lifecycle to `torque_on`. From `finalizing`, it first
  waits for `finalized`.
- `robot.shutdown`: drives the lifecycle to `finalized`.
- `robot.state`: lifecycle and joint state snapshot.
- `robot.move` and `robot.stop`: publish or zero a `MovementCommand`.
- `robot.walk_for_duration`: timed motion sequence.
- `robot.recording.start`, `robot.recording.stop`, and `robot.recording.status`:
  periodic state sampling (default interval 0.5 s).
- `system.state`: a combined snapshot of the topics, simulation, and robot.

## Resources

- `drqp://simulation.state.stream`
- `drqp://simulation.robot_state.stream`
- `drqp://simulation.world_state.stream`
- `drqp://robot.state.stream`
- `drqp://system.state.stream`

Each returns the latest event envelope.

## Contract

- The tools act on whatever ROS graph the server's environment reaches
  (`ROS_DOMAIN_ID`), whether that is the simulation or a real robot.
- The simulation PID and log live in the runtime directory: `ROS_HOME` when set.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/simulation/drqp_robot_mcp/drqp_robot_mcp/server.py:11` — server
  definition
- `packages/simulation/drqp_robot_mcp/drqp_robot_mcp/server.py:53` —
  `robot.boot`
- `packages/simulation/drqp_robot_mcp/drqp_robot_mcp/server.py:109` —
  `robot.walk_for_duration`
- `packages/simulation/drqp_robot_mcp/drqp_robot_mcp/server.py:196` —
  `robot.state.stream` resource
- `packages/simulation/drqp_robot_mcp/drqp_robot_mcp/controller.py:64` —
  `boot_up`
