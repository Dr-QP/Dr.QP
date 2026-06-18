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
- **Local scripts and docs** (`scripts/`, `docs/`) → `.github/instructions/python-local.instructions.md` — virtual env usage, notebook conventions
- **Sandbox / CI environment** → `.github/instructions/microVM-sandbox.instructions.md` — devcontainer workflows, `devcontainer exec`, colcon build/test gotchas
- **Creating agents** (`*.agent.md`) → `.github/instructions/agents.instructions.md` — frontmatter schema, tool configuration, handoffs, orchestration patterns
- **Creating skills** (`SKILL.md`) → `.github/instructions/agent-skills.instructions.md` — skill format, description best practices, bundled resources

## Temporary Files

NEVER use `$TMPDIR`. ALWAYS use `./.tmp` (relative to repo root) for temporary files; create it if it does not exist.

## Available Agents

Agents are defined in `.github/agents/` and available to Claude Code via trampolines in `.claude/agents/`. Trampolines are needed because Copilot agent frontmatter (model names, tool aliases) is incompatible with Claude Code.

- **[Principal Engineer](./.github/agents/principal-engineer.agent.md)** — architecture, design decisions, implementation strategies, PR review, TDD orchestration
- **[TDD Red](./.github/agents/tdd-red.agent.md)** — write failing tests before implementation
- **[TDD Green](./.github/agents/tdd-green.agent.md)** — implement minimal code to make tests pass
- **[TDD Refactor](./.github/agents/tdd-refactor.agent.md)** — improve code quality while keeping tests green

Consult the **Principal Engineer** agent for architecture, design decisions, and implementation strategies.

## Available Skills

Skills from `.github/skills/` are available as Claude Code slash commands via the `.claude/skills/` symlink. Claude Code discovers these automatically and exposes them as `/skill-name`:

**Git & GitHub**: `/git-commit`, `/open-pr`, `/generate-pr-description`, `/pr-feedback-resolution`, `/update-branch`, `/refine-issue`, `/extract-github-actions-logs`, `/get-codeql-data`

**ROS 2**: `/ros2-environment-setup`, `/ros2-workspace-build`, `/ros2-workspace-testing`, `/ros2-dependency-management`, `/ros2-diagnostics`, `/ros2-launch-management`, `/ros2-lifecycle-management`, `/ros2-parameter-tuning`

**Code**: `/add-test-file`, `/find-test-files`, `/code-review-standards`, `/create-ros2-package`, `/create-state-machine`, `/implement-publisher-subscriber`
