# Spec 01: Trampoline sync validation & stale hook patterns

- **Status**: proposed
- **Fixes**: AC10, AC13
- **Depends on**: —
- **Files**: `.pre-commit-config.yaml`, `py_packages/validate_agent_files/`, `AGENTS.md`
- **Size**: S

## Objective

Make Codex-trampoline drift impossible to commit and clean stale validation patterns, so that
spec 02/03 edits to `.claude/agents/` frontmatter are automatically checked against their
`.codex/agents/` counterparts. (`.cursor/agents` and both `skills` entries are symlinks into
`.claude/` and need no sync machinery; Codex trampolines are real files that duplicate the
canonical `name`/`description` by design, and only convention keeps them in sync.)

## Current behavior

- Codex trampolines in `.codex/agents/*.md` carry a verbatim copy of each canonical agent's
  `name` and `description`. The `create-agent` SKILL says "Keep `name` and `description` in
  sync with the canonical file", and `AGENTS.md` says to update trampolines "when adding or
  renaming an agent" — but no tool checks any of it (AC10). `validate_agent_files`
  (pre-commit + `validate-agent-files.yml` CI) scans only `.claude`, so a description edit in
  `.claude/agents/` lands silently while Codex keeps routing on the stale description.
- `AGENTS.md`'s sync instruction covers add/rename but not **description edits** — the exact
  unguarded case.
- The pre-commit `validate-agent-files` hook's `files` pattern includes
  `\.claude/prompts/.*\.prompt\.md` — the directory does not exist (AC13) — and does not
  trigger on trampoline edits at all.

## Target design

1. **Extend `validate_agent_files`** with a cross-catalog check:
   - every `.claude/agents/<stem>.agent.md` has a `.codex/agents/<stem>.md` trampoline;
   - trampoline `name` and `description` are byte-equal to the canonical frontmatter;
   - no orphan trampolines (a `.codex/agents/*.md` without a canonical counterpart).
2. **Fix the pre-commit hook's `files` pattern**: add `\.codex/agents/.*\.md` (so trampoline
   edits re-run validation) and drop the stale `\.claude/prompts/.*\.prompt\.md` alternative.
3. **`AGENTS.md` Catalog Locations**: extend "when adding or renaming an agent, also update
   its `.codex/agents/` trampoline" to cover description edits, e.g. "when adding or renaming
   an agent **or editing its description**, update its `.codex/agents/` trampoline to match
   (CI enforces this)."

## Out of scope

- Any content change to the agent instructions themselves (specs 02, 03).
- Skills catalogs (`.codex/skills` and `.cursor/skills` are symlinks; nothing to sync).
- `.cursor/` (symlink, follows `.claude` automatically).

## Test plan (write first)

In `py_packages/validate_agent_files/tests/`:

- Trampoline with mismatched `description` → validation error naming both files.
- Trampoline with mismatched `name` → error.
- Canonical agent without trampoline → error; orphan trampoline → error.
- Matching pair → passes.
- Current repo state passes (the four existing pairs are in sync today).

## Verification

```bash
pytest py_packages/validate_agent_files/tests
pre-commit run validate-agent-files --all-files
# mutation check: edit a description in .claude/agents/, confirm the hook now fails
```

## Acceptance criteria

- [ ] `validate_agent_files` fails on trampoline name/description drift and on
      missing/orphan trampolines, with tests.
- [ ] Pre-commit pattern covers `.codex/agents/` and drops nonexistent `.claude/prompts/`.
- [ ] `AGENTS.md` sync instruction covers description edits and mentions enforcement.
- [ ] Repo passes validation unchanged (no drift exists today).
