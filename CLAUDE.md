@AGENTS.md

# Claude Code Guidelines

## Dual-System Arrangement

This repository shares its AI tooling between **GitHub Copilot** and **Claude Code**. The canonical source of truth lives in `.github/` and is consumed by both systems:

| Artifact     | Source of truth         | Copilot location        | Claude Code location            |
| ------------ | ----------------------- | ----------------------- | ------------------------------- |
| Agents       | `.github/agents/`       | `.github/agents/`       | `.claude/agents/` (trampolines) |
| Skills       | `.github/skills/`       | `.github/skills/`       | `.claude/skills/` → symlink     |
| Instructions | `.github/instructions/` | `.github/instructions/` | Referenced from this file       |

**Always edit the `.github/` source files.** The symlink and trampolines pick up changes automatically; no duplication required.

## Catalog Locations

- **Agents**: defined in `.github/agents/`, surfaced to Claude Code via minimal trampoline files in `.claude/agents/`
- **Skills / Commands**: defined in `.github/skills/`, linked directly into Claude Code via `.claude/skills/` → `../.github/skills/` symlink
- **Instructions**: defined in `.github/instructions/`, referenced below

## Instructions

The following files in `.github/instructions/` are the detailed guidelines for this project. Read each file when working on the corresponding scope:

- **All code** → `.github/instructions/engineering.instructions.md` — shared engineering principles, SOLID, Clean Code, testing pyramid, C++/Python/ROS 2 conventions
- **Python files** (`*.py`) → `.github/instructions/python.instructions.md` — PEP 8, type hints, docstrings, exception handling, pytest
- **Launch tests** (`**/test/**/*.py` using `launch_pytest`) → `.github/instructions/launch-testing.instructions.md` — functions-only launch tests, per-process exit-code verification via `drqp_launch_testing`, fixture scope vs shutdown pattern
- **Local scripts and docs** (`scripts/`, `docs/`) → `.github/instructions/python-local.instructions.md` — virtual env usage, notebook conventions
- **Sandbox / CI environment** → `.github/instructions/microVM-sandbox.instructions.md` — devcontainer workflows, `devcontainer exec`, colcon build/test gotchas
- **Creating agents** (`*.agent.md`) → `.github/instructions/agents.instructions.md` — frontmatter schema, tool configuration, handoffs, orchestration patterns
- **Creating skills** (`SKILL.md`) → `.github/instructions/agent-skills.instructions.md` — skill format, description best practices, bundled resources
