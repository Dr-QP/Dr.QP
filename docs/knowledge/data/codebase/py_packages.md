---
type: codebase
description: Plain Python (non-ROS) packages for the workspace .venv — the Sphinx image-lightbox extension and a stale, skipped copy of the robot MCP server.
source: py_packages
source_digest: sha256:f74f2f6be62687db34326f0da0652827ba6ec49ba8078c610ebe02430d48ea6d
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
sources:
- id: code
  resource: py_packages
---

# py_packages

Python packages outside colcon. `pyproject.toml` lists `py_packages` in the root
pytest `testpaths`. `scripts/python-reformat.sh` and the reformat workflow run
ruff over it.

## Contains

[sphinxcontrib_spotlight](py_packages/sphinxcontrib_spotlight.md)

[drqp_robot_mcp](py_packages/drqp_robot_mcp.md)

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `pyproject.toml:63` — `testpaths` includes `py_packages`
- `pyproject.toml:71` — `sphinxcontrib-spotlight` editable path source
- `scripts/python-reformat.sh:10` — ruff targets `py_packages`
