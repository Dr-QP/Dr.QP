# Claude Code Customizations

This directory holds the repository's **ROS- and project-specific** AI tooling. Claude Code is the primary coding assistant; Codex (secondary) and Cursor consume these files via the symlinks in `.codex/` and `.cursor/`.

General-purpose tooling — PR workflow, git, code review, skill and agent authoring, sandbox escalation, and the Principal Engineer / TDD agents — is **not** kept here. It comes from the installed [`agentdev`](https://github.com/plume-works/agent-devcontainer) catalog and is invoked under the `agentdev:` prefix. Do not vendor a local copy of a skill the catalog already provides: a same-named local skill shadows it, and the two drift apart silently.

## Structure

```text
.claude/
├── skills/       # ROS/project skills and slash commands (<name>/SKILL.md)
├── agent-ideas/  # Draft agent specs, not yet promoted
├── hooks/        # Claude Code session hooks
└── settings.json # Claude Code permissions, marketplaces and plugin settings
```

## How it works

**Skills** (`.claude/skills/`): Claude Code discovers each `<name>/SKILL.md` as a `/name` slash command and can also invoke skills automatically based on their `description`. All bundled reference files inside each skill directory are available to Claude Code. The SKILL.md format implements the [Agent Skills](https://agentskills.io) open standard, so Codex and Cursor consume the same skills through the `.codex/skills/` and `.cursor/skills/` symlinks.

**Agents**: supplied by the `agentdev` catalog (`agentdev:Principal Engineer`, `agentdev:TDD Red`/`Green`/`Refactor`). `settings.json` declares the marketplace and enables the plugin, so they resolve without a local checkout.

**Always-on conventions** live in the repository `AGENTS.md` (Coding Conventions section) and `CLAUDE.md`; detailed task-scoped playbooks are skills.

## Editing guidance

- To add or change a **skill**: edit `.claude/skills/<name>/SKILL.md`. Use the `/agentdev:create-skill` skill, and check the result with `validate_agent_files`.
- Keep new skills ROS- or project-specific. If the need is general, contribute it upstream to the `agentdev` catalog instead.
- Never edit through the `.codex/` or `.cursor/` symlinks — treat those directories as read-only adapters.
