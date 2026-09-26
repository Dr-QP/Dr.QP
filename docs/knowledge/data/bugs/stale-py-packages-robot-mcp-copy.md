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
differ from the canonical server's namespaced tools (`robot.boot`, …). Readers
and agents can mistake it for the live implementation.

## Reproduction

1. `git ls-files py_packages/drqp_robot_mcp` lists the files.
2. `pyproject.toml` `[tool.uv.sources]` points `drqp_robot_mcp` at
   `packages/simulation/drqp_robot_mcp`.

## Root cause

When the server moved into the ROS package in #368, the older copy under
`py_packages/` was left behind. It has not changed since.

## Fix

Run `git rm -r py_packages/drqp_robot_mcp`. Before deleting, confirm that none
of its tests cover behavior missing from
`packages/simulation/drqp_robot_mcp/test/`.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `py_packages/drqp_robot_mcp/drqp_robot_mcp/server.py` — old flat tool names
- `pyproject.toml:70` — installed source is `packages/simulation/drqp_robot_mcp`
