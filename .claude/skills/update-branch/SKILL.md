---
name: update-branch
description: 'Update the current Git feature branch from a remote base branch by fetching and merging its remote-tracking ref. Use when asked to sync a feature branch, bring a branch up to date with `origin/main`, refresh a stale PR branch, or unblock CI after base-branch drift. Keywords: update branch, sync with origin/main, fetch and merge main, refresh feature branch.'
---

# Update Branch from origin/main

Fetch a remote base branch, verify whether the current feature branch is behind
it, and delegate merging and conflict resolution to
[git-merge-resolve](../git-merge-resolve/SKILL.md).

## When to Use This Skill

- Sync a feature branch with the latest `origin/main`
- Resolve branch drift causing CI failures or stale diffs
- Refresh a long-lived pull request branch before final review
- Fetch and merge a configurable remote base branch

For a merge that does not require fetching or base-branch synchronization, use
[git-merge-resolve](../git-merge-resolve/SKILL.md) directly.

## Prerequisites

- Git repository with the requested remote configured
- Clean working tree
- Current branch is not `main` or `master`
- User intent is a merge-based update, not a rebase

## Safety Rules

1. NEVER force-push as part of this workflow.
2. NEVER discard user changes without explicit permission.
3. NEVER change Git configuration or switch remotes.
4. Use local Git commands for branch updates and pushes; never update branch
   refs through a GitHub API or MCP tool.

## Bundled Script

Use [update-branch.sh](scripts/update-branch.sh) instead of running the fetch and
merge commands manually. It:

- checks repository, branch, and working-tree preconditions
- fetches the configured remote
- detects an already-current branch
- delegates the merge to the `git-merge-resolve` bundled script

Options:

- `--remote <name>` selects the remote; default: `origin`.
- `--base <branch>` selects the base branch; default: `main`.

Exit codes:

- `0`: merge completed successfully
- `1`: merge conflicts require the `git-merge-resolve` resolution workflow
- `2`: usage or preflight error
- `3`: current branch is already up to date
- `5`: current branch is a protected default branch

## Workflow 1: Run the Update Script

```bash
.claude/skills/update-branch/scripts/update-branch.sh
```

The script defaults to `origin/main`. Supply `--remote` or `--base` only when
the user requested different values.

## Workflow 2: Handle the Result

- **Exit 0**: the fetch and merge completed; the delegated
  [git-merge-resolve](../git-merge-resolve/SKILL.md) completion workflow still
  requires reformatting and targeted validation before push.
- **Exit 1**: invoke and complete the
  [git-merge-resolve](../git-merge-resolve/SKILL.md) conflict-resolution and
  completion workflows, then return here.
- **Exit 3**: report that the branch is already up to date; make no changes.
- **Exit 2 or 5**: fix the reported error before retrying. Do not discard or
  stash changes without user approval.

## Workflow 3: Push the Updated Branch

Only after the `git-merge-resolve` workflow has completed its mandatory
reformat and targeted validation:

1. Confirm the merge result:

   ```bash
   git status --short --branch
   git log --oneline --decorate -n 5
   ```

2. Push through the configured Git remote using local Git:

   ```bash
   git push origin HEAD
   ```

   Replace `origin` with the explicitly selected remote. If push authentication
   is unavailable, stop and report the blocker. Do not fall back to an API-based
   ref update.

## Completion Criteria

- The configured remote was fetched
- The remote base ref is merged into the current branch, or was already merged
- The delegated `git-merge-resolve` workflow resolved all conflicts and ran its
  required reformat and targeted validation
- The updated branch was pushed through local Git when a merge occurred

## Troubleshooting

| Issue                     | Likely Cause                   | Action                                                  |
| ------------------------- | ------------------------------ | ------------------------------------------------------- |
| Dirty working tree        | Local uncommitted changes      | Ask the user to commit or approve a stash workflow      |
| Current branch is default | Incorrect checkout             | Switch only with user authorization, then retry         |
| Conflicts reported        | Feature and base diverged      | Follow `git-merge-resolve`; do not improvise a shortcut |
| Fetch fails               | Network, remote, or auth issue | Report the error without changing the configured remote |
| Push fails                | Authentication unavailable     | Report the blocker; do not update refs through an API   |
