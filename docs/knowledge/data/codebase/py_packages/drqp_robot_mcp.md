---
type: codebase
description: A stale pre-#368 copy of the robot MCP server with flat tool names; it cannot import on its own and its tests skip as superseded.
source: py_packages/drqp_robot_mcp
source_digest: sha256:b70eadf78a52e097959744fa42d9377ecadc89f0d0b27a9a6b6722e0594909e6
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
sources:
- id: code
  resource: py_packages/drqp_robot_mcp
---

# py_packages/drqp_robot_mcp

The first FastMCP server (#325), left behind when the server moved into
[drqp_robot_mcp](../packages/simulation/drqp_robot_mcp.md) in #368. It is
tracked for deletion in
[Bug: stale copy of the robot MCP server in py_packages](../../bugs/stale-py-packages-robot-mcp-copy.md).

## Public surface

None is installed or registered: `.venv/bin/drqp_robot_mcp` comes from the ROS
package. The code defines flat tool names (`drqp_robot_boot_up`,
`drqp_robot_shut_down`, `drqp_robot_get_state`,
`drqp_robot_send_motion_command`, recording, and world state) over dataclass
models.

## Invariants & gotchas

- `controller.py` imports `.runtime`, which does not exist here.
- `tests/` calls `pytest.skip(..., allow_module_level=True)` as "superseded by
  packages/simulation/drqp_robot_mcp/test". Root `pytest` collects the tests and
  skips both.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `py_packages/drqp_robot_mcp/drqp_robot_mcp/server.py:30` —
  `drqp_robot_boot_up`
- `py_packages/drqp_robot_mcp/drqp_robot_mcp/controller.py:13` — missing
  `.runtime` import
- `py_packages/drqp_robot_mcp/tests/test_runtime.py:8` — legacy skip
