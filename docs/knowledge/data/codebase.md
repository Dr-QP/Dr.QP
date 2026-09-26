---
type: hub
description: Codebase maps derived from the code, each pinned to the tracked-source fingerprint it was read from.
stage: living
generated:
  by: human:author
  at: 2026-08-01T00:00:00Z
---

# 🧭 Codebase

*The map of the code as it actually is — written only by reading the code, never
from memory. The map mirrors the code's containment tree: one doc per component
(crate, package, module) at a canonical key matching its source path, children
linked from their parent's `## Contains` — so `iwe tree -k data/codebase`
renders the component tree. Every doc carries `source` (the code it describes),
`source_digest` (a fingerprint of that code's tracked contents), and `verified`
(the date); a digest that no longer matches the code means the doc is suspect —
refresh it. Division of truth: spec/ is what must be, architecture/ is why it's
shaped this way, this hub is what is.*

## Getting around

All ROS commands run through `scripts/with-ros-env.sh`, inside the devcontainer
(`ghcr.io/dr-qp/jazzy-ros-desktop`):

- **Install dependencies:** `scripts/ros-dep.sh`
- **Build:**
  `scripts/with-ros-env.sh colcon build --symlink-install --packages-up-to <pkg>`
  (CI: `--mixin coverage-pytest ninja rel-with-deb-info`)
- **Test:** `scripts/with-ros-env.sh colcon test --packages-select <pkg>`. The
  logs are in `log/latest_test/<pkg>/`. Set `DRQP_TEST_MODE=slow` to add the
  full Gazebo tier.
- **Lint:** `scripts/python-reformat.sh` (ruff), then
  `scripts/python-lint-check.sh` (`ament_flake8`, the CI gate).
- **Run on the robot:** `ros2 launch drqp_brain bringup.launch.py`. This is the
  deploy image's default command.
- **Run in simulation:** `ros2 launch drqp_gazebo sim.launch.py`. Add
  `load_keyboard_control:=true` for the GUI controller.
- **Agent MCP server:** `.venv/bin/drqp_robot_mcp` over stdio, after `uv sync`.

Directory → component:

| Path                                                                                                                                                                                 | Component                                                                                                                   |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------- |
| `packages/runtime/`                                                                                                                                                                  | [packages/runtime](codebase/packages/runtime.md)                                                                            |
| `packages/simulation/`                                                                                                                                                               | [packages/simulation](codebase/packages/simulation.md)                                                                      |
| `packages/vendor/`                                                                                                                                                                   | Vendored `launch` (patched `launch_pytest`) and `sdl3_vendor`. *Not mapped*                                                 |
| `packages/cmake/`                                                                                                                                                                    | Shared CMake helpers: Catch2, clang coverage, clang-format, isolated launch tests, `llvm-cov-export-all.py`. *Not mapped*   |
| `docker/ros/`                                                                                                                                                                        | [docker/ros](codebase/docker/ros.md)                                                                                        |
| `docker/act/`                                                                                                                                                                        | Image for running workflows locally with `act` (with `.actrc`, `.vars`). *Not mapped*                                       |
| `.devcontainer/`, `devcontainer-compose-pins.yml`                                                                                                                                    | [devcontainer](codebase/devcontainer.md)                                                                                    |
| `.github/`                                                                                                                                                                           | [github/workflows](codebase/github/workflows.md)                                                                            |
| `scripts/`                                                                                                                                                                           | [scripts](codebase/scripts.md)                                                                                              |
| `py_packages/`                                                                                                                                                                       | `sphinxcontrib_spotlight` (Sphinx extension, a `uv` path dep) and a stale copy of `drqp_robot_mcp`. *Not mapped*            |
| `docs/`                                                                                                                                                                              | Sphinx human docs and notebooks (`source/`), agent specs (`agents/`), this knowledge workspace (`knowledge/`). *Not mapped* |
| `hardware/`                                                                                                                                                                          | Raspberry Pi `config.txt` variants and SC16IS752 device-tree overlays. *Not mapped*                                         |
| `.claude/`, `.codex/`, `.cursor/`, `AGENTS.md`, `CLAUDE.md`, `.mcp.json`, `.vscode/`                                                                                                 | Agent and editor configuration, and project skills. *Not mapped*                                                            |
| `.iwe/`                                                                                                                                                                              | IWE config and schemas for `docs/knowledge`. *Not mapped*                                                                   |
| `pyproject.toml`, `uv.lock`, `ruff.toml`                                                                                                                                             | `.venv` tooling and editable workspace packages                                                                             |
| `.pre-commit-config.yaml`, `.prettier*`, `.clang-format`, `.editorconfig`, `.shellcheckrc`, `.hadolint.yaml`, `.ansible-lint.yml`, `zizmor.yaml`, `codecov.yml`, `.readthedocs.yaml` | Lint, format, and hosting config                                                                                            |
| `README.md`, `CONTRIBUTING.md`, `LICENSE`, `Notes.md`, `Dr.QP.code-workspace`, `.agent.metadata.json`, `.agentdev-template-progress.md`, `.gitignore`                                | Project metadata                                                                                                            |

## Components

[packages/runtime](codebase/packages/runtime.md)

[packages/simulation](codebase/packages/simulation.md)

[docker/ros](codebase/docker/ros.md)

[devcontainer](codebase/devcontainer.md)

[github/workflows](codebase/github/workflows.md)

[scripts](codebase/scripts.md)

## Flows

[flow-teleop-command](codebase/flow-teleop-command.md)

[flow-robot-lifecycle](codebase/flow-robot-lifecycle.md)

[flow-ci](codebase/flow-ci.md)

## Interfaces

*External surfaces — HTTP APIs, CLI commands, storage formats, IPC contracts —
one doc each, keyed `api-<name>`.*

[api-ros-interface](codebase/api-ros-interface.md)

[api-robot-mcp](codebase/api-robot-mcp.md)
