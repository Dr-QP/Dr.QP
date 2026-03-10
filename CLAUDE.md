# Claude Code Guidelines

## Catalog Locations

- **Claude**: `.claude/agents/`, `.claude/commands/`, `.claude/instructions/` (symlinks to `.github/`)

Update `.github` sources; symlinks pick up changes automatically.

## Key Instructions

<!-- Import shared instruction files -->

@.github/instructions/engineering.instructions.md
@.github/instructions/microVM-sandbox.instructions.md
@.github/instructions/python.instructions.md
@.github/instructions/agent-skills.instructions.md

## Agent Practices

@AGENTS.md

## Temporary Files

NEVER use `$TMPDIR`. ALWAYS use `./.tmp` (relative to repo root) for temporary files; create it if it does not exist.

## Available Agents

Agents are defined in `.github/agents/` and accessible via `.claude/agents/`.

Consult the **[Principal Engineer](/.github/agents/principal-engineer.agent.md)** agent for architecture, design decisions, and implementation strategies.

## Available Skills / Commands

Skills are defined in `.github/skills/` and accessible as Claude Code slash commands via `.claude/commands/`.

Key commands: `ros2-environment-setup`, `ros2-workspace-build`, `ros2-workspace-testing`, `ros2-dependency-management`, `code-review-standards`, `pr-feedback-resolution`, `create-ros2-package`, `add-test-file`, `implement-publisher-subscriber`, `create-state-machine`, `generate-pr-description`.
