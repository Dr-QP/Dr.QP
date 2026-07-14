---
name: git-merge-resolve
description: 'Merge a Git ref into the current branch and resolve merge conflicts with confidence-based escalation. Use when asked to merge branches or refs, finish a conflicted merge, resolve Git conflicts, or preserve both sides of divergent changes. Keywords: git merge, merge branch, merge conflict, resolve conflicts, conflicted files.'
---

# Merge a Git Ref and Resolve Conflicts

Merge a local or remote-tracking ref into the current branch, resolve conflicts
autonomously when intent is clear, and request user input when a resolution is
ambiguous.

## When to Use This Skill

- Merge a branch, tag, commit, or remote-tracking ref into the current branch
- Resolve an existing conflicted merge
- Reconcile divergent code while preserving the intent of both branches
- Provide the merge-and-resolution phase of another Git workflow

For fetching and merging the latest remote base branch into a feature branch,
use [update-branch](../update-branch/SKILL.md), which delegates its merge phase
to this skill.

## Prerequisites

- Run inside a Git repository
- Check out the branch that will receive the changes
- Start with a clean working tree unless continuing an existing conflicted merge
- Ensure the source ref exists locally; fetching refs belongs to the calling
  workflow
- Confirm the requested operation is a merge, not a rebase

## Safety Rules

1. NEVER force-push or push as part of this workflow.
2. NEVER discard user changes or abort an existing merge without explicit
   permission.
3. ALWAYS preserve branch-specific behavior and tests.
4. MANDATORY: If conflict-resolution confidence is below 70%, stop and prompt
   the user.
5. Treat the current branch as the merge destination and the supplied ref as
   the merge source; verify both before starting.

## Bundled Script

Use [git-merge-resolve.sh](scripts/git-merge-resolve.sh) to start a new merge:

```bash
.claude/skills/git-merge-resolve/scripts/git-merge-resolve.sh <source-ref>
```

Options:

- `--message <text>` sets the merge commit message.
- `-h` or `--help` displays usage.

Exit codes:

- `0`: merge completed successfully
- `1`: merge conflicts require resolution
- `2`: usage, repository, ref, or working-tree preflight failed
- `3`: source ref is already merged into `HEAD`

Do not run the script when Git already reports an in-progress merge. Continue
with the conflict-resolution workflow instead.

## Workflow 1: Inspect the Merge State

Run:

```bash
git status --short --branch
git diff --name-only --diff-filter=U
```

- If conflicted files are listed, continue with Workflow 3.
- If a merge is in progress without unresolved files, continue with Workflow 4.
- Otherwise, confirm the source ref and continue with Workflow 2.

## Workflow 2: Start the Merge

Run the bundled script with the source ref:

```bash
.claude/skills/git-merge-resolve/scripts/git-merge-resolve.sh <source-ref>
```

Handle its result:

- **Exit 0**: continue with Workflow 4 for required validation.
- **Exit 1**: continue with Workflow 3.
- **Exit 3**: report that the source is already merged; make no changes.
- **Exit 2**: fix the reported preflight error before retrying.

## Workflow 3: Resolve Conflicts

Inspect each conflicted file and the three merge stages before editing:

```bash
git diff --name-only --diff-filter=U
git diff --cc -- <path>
git show :1:<path>  # merge base
git show :2:<path>  # current branch (ours)
git show :3:<path>  # source ref (theirs)
```

Stage entries may be absent for add/delete conflicts. Use `git status` and the
available stages to determine the intended result.

### Confidence Scoring

- **90-100%**: Mechanical conflict only, such as import order, whitespace, or
  lockstep version bumps.
- **80-89%**: One side clearly supersedes the other based on nearby code and
  tests.
- **70-79%**: Small logic differences with a clear local convention and
  targeted validation.
- **Below 70%**: Behavioral ambiguity, business-rule conflict, API contract
  uncertainty, or multiple plausible outcomes.

### Resolution Heuristics

1. Preserve the destination branch's feature behavior.
2. Incorporate non-conflicting safety, compatibility, and bug-fix changes from
   the source ref.
3. Keep public interfaces stable unless the requested merge explicitly changes
   them.
4. Combine independent changes instead of choosing an entire side wholesale.
5. Update tests only when the intended merged behavior is clear; do not make
   tests conceal a behavioral regression.
6. Ensure the result compiles or imports and follows repository conventions.

### Mandatory User Escalation

If any conflicted hunk is below 70% confidence, leave the merge in progress and
ask:

```text
I can continue automatically for high-confidence conflicts, but at least one
conflict is below 70% confidence.

File: <path>
Conflict summary: <what differs>
Option A: <interpretation>
Option B: <interpretation>
Recommended: <best guess and why>

Please choose A, B, or provide a custom resolution.
```

After resolving all high-confidence conflicts, verify and stage them:

```bash
git diff --check
git diff --name-only --diff-filter=U
git add <resolved-files>
```

Do not commit while any unmerged paths remain.

## Workflow 4: Complete and Validate the Merge

1. If a merge is still in progress after all resolutions are staged, complete
   it:

   ```bash
   git commit
   ```

2. Invoke and complete the [local-reformat](../local-reformat/SKILL.md)
   workflow. Run every formatter and validation required by that skill; do not
   substitute a partial formatter command.

3. Run targeted build, test, lint, or static checks for files affected by the
   merge.

4. Confirm that the repository has no unresolved paths or conflict markers:

   ```bash
   git diff --name-only --diff-filter=U
   git grep -nE '^(<<<<<<< |=======|>>>>>>> )' || true
   git status --short --branch
   git log --oneline --decorate -n 5
   ```

5. Return control to the calling workflow. Pushing, opening a pull request, or
   performing another external action requires separate authorization or
   caller instructions.

## Conflict Types and Default Actions

| Conflict Type                | Default Action                                                  | Confidence Baseline  |
| ---------------------------- | --------------------------------------------------------------- | -------------------- |
| Whitespace / formatting only | Keep style consistent with the file                             | 95%                  |
| Import/include ordering      | Keep compilable/import-valid ordering                           | 90%                  |
| Dependency/version bumps     | Prefer the newer compatible version                             | 80%                  |
| Test expectation drift       | Preserve intended behavior, then align tests if intent is clear | 75%                  |
| Core logic divergence        | Escalate unless intent is unambiguous                           | Below 70% by default |
| API contract changes         | Escalate unless required by the requested merge                 | Below 70% by default |

## Completion Criteria

- The requested source ref is merged into the current branch, or was already
  merged
- No unmerged paths or unresolved conflict markers remain
- All conflict resolutions met the confidence threshold or were user-approved
- The mandatory local reformat workflow completed successfully after a merge
- Relevant targeted validation passes
- No push or force-push was performed by this skill

## Troubleshooting

| Issue                         | Likely Cause                    | Action                                                      |
| ----------------------------- | ------------------------------- | ----------------------------------------------------------- |
| Merge already in progress     | Earlier merge stopped           | Inspect and continue it; do not start another merge         |
| Merge aborted unexpectedly    | Interrupted workflow            | Ask before aborting or restarting                           |
| Too many ambiguous conflicts  | Widespread refactor overlap     | Escalate early and request a resolution strategy            |
| Tests fail after the merge    | Semantic drift between branches | Reconcile behavior based on confirmed intent                |
| Wrong destination checked out | Branch selection was incorrect  | Do not discard the merge; ask how the user wants to proceed |
