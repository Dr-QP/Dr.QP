---
name: update-branch
description: 'Update the current Git branch from origin by merging `origin/main` into it. Use when asked to sync a feature branch, bring branch up to date, resolve merge conflicts, or unblock CI after base branch drift. Keywords: merge `origin/main`, update branch, sync with origin/main, resolve merge conflicts.'
---

# Update Branch from origin/main

Safely update the current branch by merging `origin/main` into it, resolving conflicts autonomously when confidence is high, and requesting user input when confidence is low.

## When to Use This Skill

- Sync a feature branch with latest changes from `origin/main`
- Resolve branch drift causing CI failures or stale diffs
- Refresh long-lived PR branches before final review
- Resolve merge conflicts while preserving branch intent

## Prerequisites

- Git repository with `origin` remote configured
- Clean working tree (or user-approved stash workflow)
- Current branch is not `main`
- User intent is merge-based update (not rebase)

## Safety Rules

1. NEVER force-push as part of this workflow.
2. NEVER discard user changes without explicit permission.
3. ALWAYS preserve branch-specific behavior and tests.
4. MANDATORY: If conflict-resolution confidence is below 70%, stop and prompt the user.

## Bundled Script

Use the helper script instead of running git commands manually:

- [update-branch.sh](scripts/update-branch.sh) handles preflight checks, fetches `origin`, detects up-to-date state, and runs `git merge --no-ff origin/main`.

Options: `--remote <name>` (default: `origin`), `--base <branch>` (default: `main`).

Exit codes: `0` merged, `1` conflicts, `3` already up to date, `5` on default branch.

## Step-by-Step Workflow

### Workflow 1: Run the Script

```bash
.github/skills/update-branch/scripts/update-branch.sh
```

The script handles:
- Preflight checks (dirty tree → error, on default branch → error)
- `git fetch origin`
- Already-up-to-date detection (exit 3)
- `git merge --no-ff origin/main`
- Conflict detection and reporting (exit 1)

### Workflow 2: Handle Script Result

- **Exit 0** (merged): run targeted validation (tests/lint), then push.
- **Exit 3** (up to date): nothing to do.
- **Exit 1** (conflicts): resolve per [Workflow 3](#workflow-3-autonomous-conflict-resolution) below, then commit and push.
- **Exit 2 / 5**: fix the reported error before retrying.

### Workflow 3: Autonomous Conflict Resolution

When merge conflicts occur, resolve in this order with confidence scoring.

#### Confidence Scoring

- **90-100%**: Mechanical conflict only (import order, whitespace, lockstep version bumps).
- **80-89%**: One-side clearly supersedes the other based on nearby code and tests.
- **70-79%**: Small logic differences with clear local convention and passing targeted tests.
- **<70%**: Behavioral ambiguity, business-rule conflict, API contract uncertainty, or multiple plausible outcomes.

#### Resolution Heuristics

1. Prefer conflict blocks that preserve current branch feature behavior.
2. Incorporate non-conflicting safety or compatibility updates from `origin/main`.
3. Keep public interfaces stable unless branch changes explicitly require updates.
4. Ensure resulting code compiles and aligns with repository conventions.

#### Mandatory User Escalation

If any conflicted hunk is below 70% confidence, pause and ask the user:

```text
I can continue automatically for high-confidence conflicts, but at least one conflict is below 70% confidence.

File: <path>
Conflict summary: <what differs>
Option A: <interpretation>
Option B: <interpretation>
Recommended: <best guess and why>

Please choose A, B, or provide a custom resolution.
```

### Workflow 4: Complete Merge and Validate

1. After resolving conflicts:

   ```bash
   git add <resolved-files>
   git commit
   ```

2. Run targeted verification before push:
   - Build/test packages or modules impacted by conflict files
   - Run lint or static checks where available

3. Confirm merge result:

   ```bash
   git log --oneline --decorate -n 5
   ```

4. Push updated branch:
   ```bash
   git push origin HEAD
   ```

## Conflict Types and Default Actions

| Conflict Type                | Default Action                                                 | Confidence Baseline |
| ---------------------------- | -------------------------------------------------------------- | ------------------- |
| Whitespace / formatting only | Keep style consistent with file                                | 95%                 |
| Import/include ordering      | Keep compilable/import-valid ordering                          | 90%                 |
| Dependency/version bumps     | Prefer newer compatible version from `origin/main`             | 80%                 |
| Test expectation drift       | Preserve branch behavior, then adjust tests if intent is clear | 75%                 |
| Core logic divergence        | Escalate unless intent is unambiguous                          | <70% by default     |
| API contract changes         | Escalate unless covered by explicit branch requirement         | <70% by default     |

## Completion Criteria

- `origin/main` is merged into current branch
- All conflicts are resolved
- No unresolved markers remain:
  ```bash
  git grep -n "<<<<<<<\|=======\|>>>>>>>" || true
  ```
- Relevant validation checks pass
- Branch is pushed to `origin`

## Troubleshooting

| Issue                        | Likely Cause                    | Action                                            |
| ---------------------------- | ------------------------------- | ------------------------------------------------- |
| Merge aborted unexpectedly   | Interrupted conflict workflow   | Re-run `git merge --abort`, then restart          |
| Too many ambiguous conflicts | Widespread refactor overlap     | Escalate early; request user decision on strategy |
| Tests fail post-merge        | Semantic drift from mainline    | Fix behavior or tests based on confirmed intent   |
| Wrong branch updated         | Incorrect checkout before merge | Switch to target branch and repeat workflow       |
