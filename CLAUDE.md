@AGENTS.md

# Claude Code Guidelines

## Claude-Native AI Tooling

Claude Code is the primary coding assistant for this repository, with Codex as secondary. The canonical source of truth for all AI tooling lives in `.claude/`; other assistants consume it via trampolines and symlinks:

| Artifact | Source of truth   | Codex location                 | Cursor location             |
| -------- | ----------------- | ------------------------------ | --------------------------- |
| Agents   | `.claude/agents/` | `.codex/agents/` (trampolines) | `.cursor/agents/` → symlink |
| Skills   | `.claude/skills/` | `.codex/skills/` → symlink     | `.cursor/skills/` → symlink |

**Always edit the `.claude/` source files.** The symlinks pick up changes automatically; when adding or renaming an agent, also update its `.codex/agents/` trampoline.

## Catalog Locations

- **Agents**: defined in `.claude/agents/*.agent.md` (Claude Code subagent format), adapted for Codex via minimal trampoline files in `.codex/agents/`
- **Skills / Commands**: defined in `.claude/skills/`; each `<name>/SKILL.md` is discovered as a `/name` slash command and invoked automatically based on its `description`

## Task-Scoped Skills

Always-on coding conventions live in the Coding Conventions section of `AGENTS.md` (inlined above). Detailed task playbooks are skills — load the matching one when working on the corresponding scope:

- **Launch tests** (`**/test/**/*.py` using `launch_pytest`) → [launch-testing](.claude/skills/launch-testing/) — functions-only launch tests, per-process exit-code verification via `drqp_launch_testing`, fixture scope vs shutdown pattern
- **Sandbox / CI environment** → [microvm-sandbox](.claude/skills/microvm-sandbox/) — devcontainer workflows, `devcontainer exec`, colcon build/test gotchas
- **Creating agents** (`*.agent.md`) → [create-agent](.claude/skills/create-agent/) — frontmatter schema, tool configuration, orchestration patterns, Codex trampolines
- **Creating skills** (`SKILL.md`) → [create-skill](.claude/skills/create-skill/) — skill format, description best practices, bundled resources
