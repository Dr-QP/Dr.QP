# Agent catalog remediation specs

Implementation-ready specifications fixing inefficiencies, redundancy, and contradictions found
in an in-depth review (2026-07-11) of the agent instruction files: `.claude/agents/*.agent.md`,
`.codex/agents/*.md`, `AGENTS.md`, and the skills that reference them (`create-agent`,
`pr-feedback-resolution`, `code-review-standards`, `generate-pr-description`).

The headline finding: **the three TDD sub-agents are C#/.NET boilerplate** (xUnit,
FluentAssertions, NuGet, Azure Key Vault, Serilog, `Span<T>`) **in a workspace whose own
conventions mandate pytest and ROS 2 / colcon workflows**, and their interaction model
(blocking on user confirmation, writing to GitHub issues) contradicts how sub-agents actually
run. The Principal Engineer agent duplicates guidance that already lives in skills.

## Findings index

IDs `AC1`…`AC14` are referenced by the specs below. Severity: **C** = contradiction,
**R** = redundancy, **I** = inefficiency.

- **AC1 (C)** — TDD Red/Green/Refactor prescribe a C#/.NET stack — xUnit, FluentAssertions,
  AutoFixture, `List<T>`, NuGet scanning, Azure Key Vault, `IOptions`, Serilog, `Span<T>`,
  SonarQube/Checkmarx, web-app OWASP/XSS/SQL checklists — contradicting `AGENTS.md` ("Always
  use pytest — never unittest"; C++ Core Guidelines) in a repo with no C# at all.
- **AC2 (C)** — The TDD agents never mention `scripts/with-ros-env.sh`, `colcon build/test`,
  `launch_pytest`, or the repo's test-scaffolding skills — there is no repo-correct way, per
  their own instructions, to run the tests they are told to write and verify.
- **AC3 (C)** — All three TDD agents order: "Confirm your plan with the user … NEVER start
  making changes without user confirmation." Sub-agents cannot reach the user (`create-agent`
  SKILL: "the agent starts cold … does not see the parent conversation"), so this deadlocks
  the Principal Engineer's mandated autonomous Red→Green→Refactor orchestration.
- **AC4 (C)** — TDD Red's only requirements source is a GitHub issue found by extracting
  digits from the branch name (`*{number}*`). Spec-driven branches (e.g.
  `locomotion-spec-02-time-based-gait-targets`) match the wrong number and have no issue;
  `docs/agents/specs/` — which names these agents as its workflow — is never consulted.
- **AC5 (C)** — TDD Green/Refactor autonomously comment on and close GitHub issues
  (outward-facing writes, no gate) while AC3 simultaneously forbids ungated local file edits —
  the caution is inverted.
- **AC6 (C)** — `pr-feedback-resolution` SKILL contradicts itself — "Never invoke tdd-red,
  tdd-green, or tdd-refactor agents directly" (When to Delegate) vs "Use TDD Red / TDD Green /
  TDD Refactor" (Codecov workflow) — and delegates to `task-researcher` and `task-planner`
  agents that do not exist in any catalog.
- **AC7 (C)** — `AGENTS.md` best practice #11 mandates the `vscode/askQuestions` tool — a
  non-Claude tool alias, the exact mistake the `create-agent` SKILL lists under "Common
  Mistakes to Avoid" (Claude Code's equivalent is `AskUserQuestion`).
- **AC8 (R)** — Principal Engineer inlines ~50 lines of PR-description and code-review
  guidance duplicating the `code-review-standards`, `generate-pr-description`, and `pr-review`
  skills — skills that themselves say "do not restate broad engineering rules." The agent
  references only one skill (`ros2-environment-setup`).
- **AC9 (R)** — Principal Engineer's "Planning Workflow" and "Complex Task Planning" sections
  repeat each other; its TDD phase bullets restate the three sub-agents' frontmatter
  descriptions verbatim in substance.
- **AC10 (R)** — Codex trampolines duplicate each canonical description verbatim with no
  automated equality check (`validate_agent_files` only scans `.claude`), so description
  edits silently drift out of sync.
- **AC11 (I)** — Principal Engineer's "When to Use This Agent / AUTOMATICALLY TRIGGER"
  section addresses the *caller*, but delegation decisions read only the frontmatter
  `description` — the section is dead weight in every run. Its description is also 263 chars,
  over the repo's own 50–250 guideline.
- **AC12 (I)** — The three TDD leaf agents carry the `Agent` tool. They are terminal workers
  in the orchestration; granting `Agent` invites recursive spawning and violates the
  least-privilege rule in `create-agent`.
- **AC13 (I)** — `.pre-commit-config.yaml` `validate-agent-files` hook watches
  `.claude/prompts/.*\.prompt\.md` — a path that does not exist (stale pattern).
- **AC14 (I)** — `docs/agents/specs/README.md` links `todo-remediation/README.md`, which does
  not exist.

## Implementation order

Specs are numbered in implementation order. 01 lands first so that later description edits in
`.claude/agents/` cannot silently drift out of sync with the Codex trampolines.

| Spec                                              | Title                                            | Fixes                         | Depends on |
| ------------------------------------------------- | ------------------------------------------------ | ----------------------------- | ---------- |
| [01](01-catalog-integrity-and-sync-validation.md) | Trampoline sync validation & stale hook patterns | AC10, AC13                    | —          |
| [02](02-retarget-tdd-agents-to-this-workspace.md) | Retarget TDD agents to this workspace            | AC1, AC2, AC3, AC4, AC5, AC12 | 01 (soft)  |
| [03](03-principal-engineer-dedup-and-slim.md)     | Principal Engineer: deduplicate & slim           | AC8, AC9, AC11                | 01 (soft)  |
| [04](04-repair-dangling-cross-references.md)      | Repair dangling cross-references                 | AC6, AC7, AC14                | —          |

## Conventions for implementing agents

- **One spec per PR.** Keep diffs reviewable.
- **Canonical source is `.claude/`** (`AGENTS.md` Catalog Locations); `.cursor/agents` and
  both `skills` entries are symlinks and follow automatically. Until spec 01 lands, keep the
  `.codex/agents/` trampoline `name`/`description` in sync by hand when they change.
- **Validate** with `validate_agent_files --recommend .claude` (pre-commit runs `--ci`).
- **Do not grow the files.** Agent prompts are paid for on every invocation; the remediation
  direction is shorter, repo-specific instructions that point at skills, not longer ones.
