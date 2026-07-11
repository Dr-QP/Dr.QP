# Spec 04: Repair dangling cross-references

- **Status**: proposed
- **Fixes**: AC6, AC7, AC14
- **Depends on**: —
- **Files**: `.claude/skills/pr-feedback-resolution/SKILL.md`, `AGENTS.md`,
  `docs/agents/specs/README.md`
- **Size**: S

## Objective

Remove references to agents and tools that do not exist, and resolve one skill's
self-contradiction about who may invoke the TDD agents, so instruction files never send an
agent down a path that cannot be followed.

## Current behavior

- **AC6 — `pr-feedback-resolution` contradicts itself and references ghosts.**
  - "When to Delegate" (l.33–34): "Delegate to Principal Engineer agent for all code
    implementation … **Never invoke** tdd-red, tdd-green, or tdd-refactor agents directly."
  - Codecov workflow (l.234–236): "Use TDD Red: Write failing test … Use TDD Green … Use TDD
    Refactor" — with no mention that this must route through the Principal Engineer. An agent
    reading only the Codecov section (the likely entry point when fixing patch coverage) gets
    the opposite instruction from l.34.
  - l.35–36 delegate to `task-researcher` and `task-planner` — no such agents exist in
    `.claude/agents/`, `.codex/agents/`, or the built-in set. A follower either errors out or
    silently substitutes something.
- **AC7 — `AGENTS.md` best practice #11** mandates `vscode/askQuestions` "for all yes/no and
  multiple-choice user questions". That is another assistant's tool alias; it does not exist
  in Claude Code (whose equivalent is `AskUserQuestion`) — precisely the mistake the
  `create-agent` SKILL warns against ("Referencing tools that don't exist in Claude Code —
  tool aliases from other assistants"). Since `AGENTS.md` is the shared instruction file for
  *all* assistants, tool-specific names need per-assistant qualification or neutral wording.
- **AC14 — `docs/agents/specs/README.md`** programs table links
  `todo-remediation/README.md`; the directory does not exist (only `locomotion/` and
  `roadmap/` do). Agents told to pick up todo-remediation work have nothing to read.

## Target design

1. `pr-feedback-resolution` Codecov section: rephrase steps to "Delegate to the Principal
   Engineer to run the TDD cycle (Red: failing test for uncovered path; Green: …)", making it
   consistent with "When to Delegate". Remove or replace the `task-researcher`/`task-planner`
   bullets: either point at real equivalents (the built-in `Explore` and `Plan` agent types)
   or drop the bullets and fold the guidance into "research/plan before implementing".
2. `AGENTS.md` #11: reword to assistant-neutral with per-assistant mapping, e.g. "For yes/no
   and multiple-choice questions, prefer the assistant's structured-question tool
   (VS Code Copilot: `vscode/askQuestions`; Claude Code: `AskUserQuestion`) over free-text."
3. `docs/agents/specs/README.md`: either commit the missing `todo-remediation/` program
   README (if the audit output exists somewhere) or drop the row and the "three programs"
   phrasing until it does. Prefer whichever is truthful at implementation time; do not leave
   a dead link.
4. While in `AGENTS.md`: the closing line "Always edit the `AGENTS.md`, not just `CLAUDE.md`"
   predates `CLAUDE.md` becoming a pure `@AGENTS.md` include; reword to "Edit `AGENTS.md`;
   `CLAUDE.md` only includes it" (drive-by, no behavior change).

## Out of scope

- Creating `task-researcher`/`task-planner` agents (nothing currently justifies them; if a
  real need emerges, that is its own spec via the `create-agent` skill).
- Writing the todo-remediation program content itself.

## Test plan (write first)

- Doc-lint/grep checks (add alongside spec 01's validator tests if that lands first):
  - no occurrence of `task-researcher|task-planner` under `.claude/`;
  - every agent name mentioned in `.claude/skills/*/SKILL.md` resolves to a file in
    `.claude/agents/` or a known built-in agent type;
  - no relative links in `docs/agents/specs/README.md` pointing at missing files (a link
    checker over `docs/agents/specs/` is sufficient).

## Verification

```bash
grep -rn "task-researcher\|task-planner" .claude/ && echo FAIL || echo OK
grep -rn "vscode/askQuestions" AGENTS.md   # only inside the per-assistant mapping
pre-commit run --all-files
```

Read `pr-feedback-resolution` end-to-end once: every "delegate to X" names something that
exists, and the TDD-invocation rule reads the same in both sections.

## Acceptance criteria

- [ ] `pr-feedback-resolution` gives one consistent TDD-delegation rule; no references to
      nonexistent agents.
- [ ] `AGENTS.md` #11 names each assistant's real tool; no bare foreign tool alias.
- [ ] `docs/agents/specs/README.md` has no dead program links.
- [ ] `AGENTS.md` closing note reflects that `CLAUDE.md` is an include.
