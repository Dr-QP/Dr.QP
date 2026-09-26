---
type: bug
description: py_packages/drqp_robot_mcp is an older, manifest-less copy of the MCP server that nothing installs and that ruff still formats.
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T12:00:00Z
sources:
- id: copy
  resource: py_packages/drqp_robot_mcp
- id: canonical
  resource: packages/simulation/drqp_robot_mcp
---

# Bug: stale copy of the robot MCP server in py_packages

## Symptom

`py_packages/drqp_robot_mcp/` holds `server.py`, `controller.py`, `models.py`,
and tests, but no `pyproject.toml`. Its tool names (`drqp_robot_boot_up`, …)
differ from the canonical server's namespaced tools (`robot.boot`, …).
`controller.py` imports `.runtime`, which the copy does not contain, so the
package cannot even be imported on its own. Readers and agents can still mistake
it for the live implementation.

## Reproduction

1. `git ls-files py_packages/drqp_robot_mcp` lists the files.
2. `pyproject.toml` `[tool.uv.sources]` points `drqp_robot_mcp` at
   `packages/simulation/drqp_robot_mcp`.
3. `pytest py_packages/drqp_robot_mcp -rs` shows both test modules skipped:
   "Legacy tests under py_packages/drqp_robot_mcp are superseded by
   packages/simulation/drqp_robot_mcp/test." The root `pytest` collects
   `py_packages` through `testpaths`.

## Root cause

When the server moved into the ROS package in #368, the older copy under
`py_packages/` was left behind. It has not changed since.

## Fix

Run `git rm -r py_packages/drqp_robot_mcp`. Its tests already skip themselves as
superseded, so no coverage is lost.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `py_packages/drqp_robot_mcp/drqp_robot_mcp/server.py` — old flat tool names
- `py_packages/drqp_robot_mcp/drqp_robot_mcp/controller.py:13` — imports the
  missing `.runtime`
- `py_packages/drqp_robot_mcp/tests/test_runtime.py:8` — module-level legacy
  skip
- `pyproject.toml:63` — root `pytest` `testpaths` includes `py_packages`
- `pyproject.toml:70` — installed source is `packages/simulation/drqp_robot_mcp`
