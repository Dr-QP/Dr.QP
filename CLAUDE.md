# Claude Code Guidelines

## Catalog Locations

- **Claude**: `.claude/agents/`, `.claude/commands/`, `.claude/instructions/` (symlinks to `.github/`)

Update `.github` sources; symlinks pick up changes automatically.

## Key Instructions

<!-- Import shared instruction files -->

@.claude/instructions/engineering.instructions.md
@.claude/instructions/microVM-sandbox.instructions.md
@.claude/instructions/python.instructions.md
@.claude/instructions/agent-skills.instructions.md

## Temporary Files

NEVER use `$TMPDIR`. ALWAYS use `./.tmp` (relative to repo root) for temporary files; create it if it does not exist.

## Available Agents

Agents are defined in `.github/agents/` and accessible via `.claude/agents/`.

Consult the **[Principal Engineer](./.github/agents/principal-engineer.agent.md)** agent for architecture, design decisions, and implementation strategies.
