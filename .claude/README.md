# Claude Code Customizations

This directory provides Claude Code access to the shared AI tooling defined in `.github/`. The canonical source of truth is always in `.github/` — both GitHub Copilot and Claude Code consume from there.

## Structure

```text
.claude/
├── agents/       # Trampoline subagent files → .github/agents/
├── skills/       # Symlink → ../.github/skills/ (skills and slash commands)
├── hooks/        # Claude Code session hooks
└── settings.json # Claude Code permissions and plugin settings
```

## How it works

**Skills** (`.claude/skills/`): A directory symlink pointing to `../.github/skills/`. Claude Code discovers each `<name>/SKILL.md` as a `/name` slash command and can also invoke skills automatically based on their `description`. All bundled scripts and reference files inside each skill directory are available to Claude Code. The SKILL.md format is shared between Copilot and Claude Code (both implement the [Agent Skills](https://agentskills.io) open standard).

**Agents** (`.claude/agents/*.md`): Trampoline files — minimal wrappers with Claude Code-compatible frontmatter (name, description, tool list) that delegate to the corresponding `.github/agents/*.agent.md` for the full instructions. Trampolines are needed because Copilot agent frontmatter uses incompatible model names (`GPT-5.4`) and tool aliases (`context7/*`, `findTestFiles`, etc.).

**Instructions**: Referenced directly in `CLAUDE.md` by file path. No duplication — edit `.github/instructions/` to update both systems.

## Editing guidance

- To add or change a **skill**: edit `.github/skills/<name>/SKILL.md`. The symlink means Claude Code picks it up immediately (no restart needed if the `skills/` directory already exists).
- To add a **new skill**: create `.github/skills/<new-name>/SKILL.md`. The symlink covers it automatically.
- To add or change an **agent**: edit `.github/agents/*.agent.md`; update the corresponding `.claude/agents/*.md` trampoline only if the name, description, or tool list changes.
- To add or change **instructions**: edit `.github/instructions/*.instructions.md` and add a reference line to `CLAUDE.md` if the file is new.
