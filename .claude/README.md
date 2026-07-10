# Claude Code Customizations

This directory is the **canonical source of truth** for the repository's AI tooling. Claude Code is the primary coding assistant; Codex (secondary) and Cursor consume these files via trampolines and symlinks in `.codex/` and `.cursor/`.

## Structure

```text
.claude/
├── agents/       # Subagent specs (*.agent.md, Claude Code frontmatter)
├── skills/       # Skills and slash commands (<name>/SKILL.md)
├── agent-ideas/  # Draft agent specs not yet promoted to agents/
├── hooks/        # Claude Code session hooks
└── settings.json # Claude Code permissions and plugin settings
```

## How it works

**Skills** (`.claude/skills/`): Claude Code discovers each `<name>/SKILL.md` as a `/name` slash command and can also invoke skills automatically based on their `description`. All bundled scripts and reference files inside each skill directory are available to Claude Code. The SKILL.md format implements the [Agent Skills](https://agentskills.io) open standard, so Codex and Cursor consume the same skills through the `.codex/skills/` and `.cursor/skills/` symlinks.

**Agents** (`.claude/agents/*.agent.md`): Full subagent definitions with Claude Code frontmatter (`name`, `description`, `tools`, optional `model`). Codex uses minimal trampoline files in `.codex/agents/` that delegate to these specs; Cursor reads them via the `.cursor/agents/` symlink.

**Always-on conventions** live in the repository `AGENTS.md` (Coding Conventions section) and `CLAUDE.md`; detailed task-scoped playbooks are skills.

## Editing guidance

- To add or change a **skill**: edit `.claude/skills/<name>/SKILL.md`. See the [create-skill](skills/create-skill/SKILL.md) skill.
- To add or change an **agent**: edit `.claude/agents/*.agent.md` and keep the matching `.codex/agents/*.md` trampoline's `name`/`description` in sync. See the [create-agent](skills/create-agent/SKILL.md) skill.
- Never edit through the `.codex/` or `.cursor/` symlinks — treat those directories as read-only adapters.
