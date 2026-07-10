@AGENTS.md

# Claude Code Guidelines

## Claude-Native AI Tooling

Claude Code is the primary coding assistant for this repository, with Codex as secondary. The canonical source of truth for all AI tooling lives in `.claude/`; other assistants consume it via trampolines and symlinks:

| Artifact     | Source of truth         | Codex location                    | Cursor location               |
| ------------ | ----------------------- | --------------------------------- | ----------------------------- |
| Agents       | `.claude/agents/`       | `.codex/agents/` (trampolines)    | `.cursor/agents/` → symlink   |
| Skills       | `.claude/skills/`       | `.codex/skills/` → symlink        | `.cursor/skills/` → symlink   |
| Instructions | `.claude/instructions/` | `.codex/instructions/` → symlink  | `.cursor/rules/` → symlink    |

**Always edit the `.claude/` source files.** The symlinks pick up changes automatically; when adding or renaming an agent, also update its `.codex/agents/` trampoline.

## Catalog Locations

- **Agents**: defined in `.claude/agents/*.agent.md` (Claude Code subagent format), adapted for Codex via minimal trampoline files in `.codex/agents/`
- **Skills / Commands**: defined in `.claude/skills/`; each `<name>/SKILL.md` is discovered as a `/name` slash command and invoked automatically based on its `description`
- **Instructions**: defined in `.claude/instructions/`, referenced below

## Instructions

The following files in `.claude/instructions/` are the detailed guidelines for this project. Read each file when working on the corresponding scope:

- **All code** → `.claude/instructions/engineering.instructions.md` — shared engineering principles, SOLID, Clean Code, testing pyramid, C++/Python/ROS 2 conventions
- **Python files** (`*.py`) → `.claude/instructions/python.instructions.md` — PEP 8, type hints, docstrings, exception handling, pytest
- **Launch tests** (`**/test/**/*.py` using `launch_pytest`) → `.claude/instructions/launch-testing.instructions.md` — functions-only launch tests, per-process exit-code verification via `drqp_launch_testing`, fixture scope vs shutdown pattern
- **Local scripts and docs** (`scripts/`, `docs/`) → `.claude/instructions/python-local.instructions.md` — virtual env usage, notebook conventions
- **Sandbox / CI environment** → `.claude/instructions/microVM-sandbox.instructions.md` — devcontainer workflows, `devcontainer exec`, colcon build/test gotchas
- **Creating agents** (`*.agent.md`) → `.claude/instructions/agents.instructions.md` — frontmatter schema, tool configuration, orchestration patterns, Codex trampolines
- **Creating skills** (`SKILL.md`) → `.claude/instructions/agent-skills.instructions.md` — skill format, description best practices, bundled resources
