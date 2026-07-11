# Spec 03: Principal Engineer — deduplicate & slim

- **Status**: proposed
- **Fixes**: AC8, AC9, AC11
- **Depends on**: 01 (soft — without it, keep the `.codex/agents/` trampoline description in
  sync by hand)
- **Files**: `.claude/agents/principal-engineer.agent.md`
  (+ `.codex/agents/principal-engineer.md` if the description changes)
- **Size**: S

## Objective

Cut the Principal Engineer prompt from 142 lines to its unique value — orchestration,
judgment, and repo-specific direction — by delegating everything already owned by a skill,
removing caller-facing routing text, and collapsing internal repetition. Shorter prompt, same
(or better) behavior, lower per-invocation cost, and one owner per rule.

## Current behavior

`principal-engineer.agent.md`:

- **AC8 — duplicates skills.** "Pull Request Creation & Review" + "PR Description Guidance"
  (~35 lines, l.89–123) restate what `code-review-standards`, `generate-pr-description`, and
  `pr-review` own — change analysis, message-accuracy skepticism, testing-strategy/migration/
  risk sections. Those skills declare AGENTS.md the source of truth and explicitly avoid
  restating shared rules; the agent restates them anyway, so the two copies can (and will)
  diverge — e.g. the skills' concrete rules ("imperative mood", "avoid tables", the PR
  template) appear nowhere in the agent's parallel list. The agent references exactly one
  skill (`ros2-environment-setup`) out of the 30+ available.
- **AC9 — internal repetition.** "Planning Workflow" (l.36–44) and "Complex Task Planning"
  (l.45–51) say the same thing twice (plan before implementing, TodoWrite items, re-plan on
  change). The "TDD Phase Orchestration" bullets restate each sub-agent's frontmatter
  description (which the orchestrator already sees in its agent list).
- **AC11 — dead routing text.** "When to Use This Agent / AUTOMATICALLY TRIGGER THIS AGENT
  FOR" (l.11–23) addresses the _caller_, but delegation is decided from the frontmatter
  `description` alone — the body is only read _after_ selection, making the section pure
  overhead in every run. The `description` itself is 263 chars, over the repo's own 50–250
  guideline (`create-agent` SKILL).
- Also caller-facing-in-the-wrong-place: "**MUST** offer to create GitHub Issues" for tech
  debt — as a sub-agent it cannot "offer" to the user mid-run; the offer belongs in its final
  report (same class of defect as spec 02's AC3, fixed here for this file).

## Target design

1. **Delete** "When to Use This Agent" (l.11–23). Fold any trigger worth keeping into the
   frontmatter `description`, and trim that description to ≤250 chars.
2. **Replace** the PR sections with two short paragraphs that _invoke_ the skills: reviews →
   `pr-review` + `code-review-standards`; descriptions/creation → `generate-pr-description` +
   `open-pr`. Keep only the judgment additions a principal brings that the skills lack (e.g.
   "code is the source of truth over commit messages" if it is absent from the skills — if it
   is worth keeping, move it _into_ `code-review-standards` and reference it).
3. **Merge** "Planning Workflow" and "Complex Task Planning" into one section. In the TDD
   orchestration list keep phase order, context-passing, and verify-between-phases; drop the
   restated sub-agent descriptions.
4. **Tech debt**: "recommend GitHub Issues in your final report (`gh issue create` command
   ready to run)" instead of "offer to create".
5. **Add** the pointers the agent actually needs in this repo (one line each): build/test via
   `scripts/with-ros-env.sh` (or the `ros2-workspace-build`/`ros2-workspace-testing` skills),
   `python-format-lint`, `launch-testing`. Net line count must go **down** — target ≤ 90
   lines.

## Out of scope

- TDD sub-agent content (spec 02).
- Changing the skills themselves; if guidance must move (item 2), it moves verbatim in the
  same PR, not rewritten.

## Test plan (write first)

- `validate_agent_files --recommend .claude` passes; description ≤ 250 chars.
- Doc-lint/grep check: PR-description structural rules (summary/why/testing/migration lists)
  appear in exactly one place under `.claude/` (the skills), not in the agent.
- All relative skill links from the agent resolve.

## Verification

```bash
validate_agent_files --recommend .claude
pre-commit run validate-agent-files --all-files
wc -l .claude/agents/principal-engineer.agent.md   # ≤ 90
```

Dry run: ask the Principal Engineer to review a small PR — it should invoke
`pr-review`/`code-review-standards` rather than free-styling a parallel rubric, and its tech
debt suggestions should arrive as report items, not questions to an absent user.

## Acceptance criteria

- [ ] "When to Use This Agent" section removed; description ≤ 250 chars and carries the
      triggers.
- [ ] PR review/description guidance lives only in the skills; agent references them.
- [ ] Single planning section; no restated sub-agent descriptions.
- [ ] No mid-run "offer to the user" steps.
- [ ] File ≤ 90 lines; validator and pre-commit pass; trampoline description in sync.
