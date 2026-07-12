---
name: implement-agent-specs
description: 'Implement numbered, implementation-ready specs from docs/agents/specs one at a time, respecting their dependencies and producing one stacked pull request per spec. Use when asked to implement, continue, or ship a specs program such as locomotion, roadmap, or formatting-linting. Keywords: agent specs, implementation specs, docs/agents/specs, spec program, stacked PR, one spec per PR.'
---

# Implement Agent Specs

Implement a selected program under `docs/agents/specs/` as a sequence of small,
independently reviewable, stacked pull requests. One numbered spec produces exactly
one branch, commit, and pull request.

## When to Use This Skill

Use this skill when the user provides a spec program name (for example,
`locomotion`, `roadmap`, or `formatting-linting`) and asks to implement the next
spec, a numbered range, or the program in order.

Do not use it for an unnumbered design document, a one-off implementation request,
or a request to edit the specs without implementing them.

## Inputs and Selection

Treat the program name as the directory name below `docs/agents/specs/`. Read the
root specs README, the selected program README, and the candidate spec before
changing code.

- With no spec number, select the lowest-numbered spec file still present in the
  program directory.
- With a requested number or range, process only that selection, in numeric order.
- Treat a missing earlier numbered spec as landed only when the program README says
  it landed or otherwise records it as complete. Do not silently skip a missing,
  undocumented prerequisite.
- Check the program dependency table and the candidate spec's own prerequisites.
  Stop and report the unmet dependency; do not implement around it.
- Preserve the program README's ordering even when independent dependency branches
  would permit parallel work. Numeric program order is authoritative for this
  workflow.

When the request is ambiguous about whether to continue, complete only the next
eligible spec. Continue to another spec only when the user requests it, either in
the initial range or in a later message.

## Coordinator and Worktree Rules

For one requested spec, a single implementing agent may perform the workflow.

For two or more requested specs, the main agent is a coordinator only. Assign one
implementing subagent to each spec. Dispatch them sequentially when dependencies
or stack order require it; do not let two agents edit the shared checkout or the
same branch concurrently.

Give each implementation agent a separate Git worktree below `./.tmp/` and its
own branch. Create `./.tmp/` first if necessary. Never use `$TMPDIR`. The agent
for spec _N+1_ starts only after spec _N_ has committed successfully, and bases its
branch on spec _N_'s branch. The coordinator verifies the prior commit, stack base,
and pull-request number before dispatching the next agent.

The coordinator must not implement production changes, tests, or documentation for
an assigned spec. It selects work, prepares isolated worktrees, provides each agent
its spec path and base branch, tracks the resulting PR, and reports status.

## Per-Spec Workflow

1. Create a descriptive branch, such as
   `spec/<program>-<number>-<short-slug>`. Base the first branch on the requested
   base branch (normally `origin/main`). Base every later branch on the immediately
   preceding spec branch; do not flatten, squash, or rebase the stack without an
   explicit request.
2. Read the full spec, its dependency specs as needed, the program README, and
   repository `AGENTS.md`. Identify the owning packages, acceptance criteria,
   behavior changes, and the spec's **Test plan (write first)**.
3. Implement with TDD: write and demonstrate the specified failing test, make the
   minimal implementation pass, then refactor while keeping tests green. Apply the
   repository's relevant language, ROS, launch-testing, formatting, and build/test
   skills. Use `scripts/with-ros-env.sh` for ROS commands and the narrowest package
   selectors; use the required sandbox escalation for ROS build, test, and lint.
4. Run the spec's required validation and any directly affected focused checks.
   Record the actual commands and results. Do not claim unrun checks passed.
5. Resolve the documentation only after implementation and validation succeed:
   update the owning program `README.md` to mark the spec's listed findings as
   resolved/landed, including a PR number only when it is known. Follow any
   program-specific instructions for additional finding indexes. Remove the
   completed numbered spec file in the same commit. Keep the program ordering table
   accurate so the next invocation selects the right spec.
6. Review the staged diff against this spec's stack base, not `origin/main` for a
   later stacked branch. Ensure it contains only the current spec's implementation,
   tests, status update, and deleted spec file. Preserve the inherited commits from
   predecessor branches unchanged.
7. Stage only those files and create one conventional commit. Use the
   [`git-commit`](../git-commit/SKILL.md) skill to derive the message from the staged
   diff.
8. Push using local Git commands only. Never use a GitHub API or MCP tool to move
   branch refs or push branch contents. Then create a pull request with the
   [`generate-pr-description`](../generate-pr-description/SKILL.md) and
   [`open-pr`](../open-pr/SKILL.md) workflows.

## Stacked Pull Requests

Create the first PR against `main` (or the requested base). Create every subsequent
PR against the immediately preceding spec branch, so its review diff shows only the
new spec. Confirm the PR base before creation.

Each PR title and body must discuss only its own spec. In the body, add one compact
stack note, for example:

```markdown
Stacked on: #123, #124
```

List every predecessor PR number in stack order. Do not repeat predecessor changes,
include a combined program summary, or claim that the whole program is complete.

If the PR number is required to mark a finding resolved, create the PR after the
code commit, then make a small follow-up commit on the same branch that records the
actual number before updating the PR. That follow-up remains part of the same
per-spec PR and must contain no other work.

## Completion and Blockers

Report each completed spec with its branch, commit, PR URL, stack base, resolved
findings, removed spec file, and validation results. Then either move to the next
explicitly requested spec or stop.

Stop and ask for direction when a required dependency is neither landed nor in the
requested stack, a test exposes an out-of-scope design decision, the working tree
contains overlapping user changes, push authentication is unavailable, or PR
creation fails. Do not mark findings resolved or delete a spec file when its
implementation or required validation is incomplete.

## Validation Checklist

- [ ] The skill name and directory are `implement-agent-specs`.
- [ ] The description contains what the skill does, when it triggers, and spec/PR keywords.
- [ ] A request for multiple specs uses one subagent and isolated worktree per spec.
- [ ] Every implementation uses numeric order, dependency checks, TDD, and focused validation.
- [ ] Every completed spec is a distinct stacked branch, commit, and PR.
- [ ] The program README marks only actually resolved findings and the completed spec file is deleted.
- [ ] `validate_agent_files --ci .claude` passes.
