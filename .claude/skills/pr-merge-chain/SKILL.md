---
name: pr-merge-chain
description: 'Merge a stacked chain of GitHub pull requests in dependency order. Use when a top PR or branch contains earlier open PRs (for example main → PR-1 → PR-2 → PR-3), when asked to merge a PR stack/chain, or to reconcile stacked branches after squash merges. Keywords: merge PR chain, merge stacked PRs, PR stack, dependent PRs, squash merge chain, pr-merge-chain.'
license: MIT
---

# Merge a Pull-Request Chain

Merge a linear stack of open pull requests, starting with the PR nearest the
repository's default branch. Delegate each individual GitHub merge to a
sub-agent that follows the [pr-merge](../pr-merge/SKILL.md) skill. Between
merges, locally reconnect the squashed history and push only the repaired
successor branch, so each remaining PR continues cleanly from the new default
branch.

## When to Use This Skill

- A PR branch includes one or more open predecessor PR branches.
- Someone asks to merge a PR stack, chain, or a top PR and all of its
  dependencies.
- A stacked PR chain must be reconciled after each squash merge.

Do not use this skill for unrelated PRs, a graph with branching dependencies,
or a chain whose lowest PR does not target the repository's default branch.
Report those cases and ask for the intended merge order instead.

## Input and Safety Boundaries

Accept exactly one input: the number, URL, or head branch of the top PR.

- Resolve it with `gh pr view <input>`. It must be open and not a draft.
- Require `gh auth status` to succeed and identify the repository default
  branch with `gh repo view --json defaultBranchRef`.
- Use normal local Git commands and `gh` commands. Do not use GitHub API or
  GitHub MCP tools to update refs, and never force-push.
- Preserve the caller's working tree. Coordinate Git history in a dedicated
  temporary worktree under `./.tmp/`; never use `$TMPDIR`.
- Do not begin merging until the complete, linear chain and all its PR states
  have been verified. Stop on a closed, draft, missing, or non-linear member.

## Discover and Validate the Chain

1. Fetch the latest remote refs and create an isolated detached worktree, for
   example `./.tmp/pr-merge-chain-<top-pr-number>`. Fetch every candidate PR's
   `refs/pull/<number>/head` into a dedicated local branch in that worktree.
   Do this before the first merge: GitHub commonly deletes a PR's remote head
   branch once it is merged.

   ```bash
   git fetch --prune origin
   git worktree add --detach ./.tmp/pr-merge-chain-<top> origin/<default-branch>
   git -C ./.tmp/pr-merge-chain-<top> fetch origin \
     +refs/pull/<number>/head:refs/heads/pr-merge-chain/<top>/<number>
   ```

   These private `pr-merge-chain/...` branches are disposable coordinator
   refs. Never overwrite the user's ordinary local branches.

2. List all open PRs with their number, URL, head branch, head SHA, base
   branch, and draft state. Combine that data with Git ancestry rather than
   trusting PR numbers or branch names. A candidate belongs to the chain when
   its fetched head is an ancestor of the supplied top PR head:

   ```bash
   git -C <worktree> merge-base --is-ancestor \
     pr-merge-chain/<top>/<candidate> pr-merge-chain/<top>/<top>
   ```

3. Sort the candidates from the fewest commits ahead of
   `origin/<default-branch>` to the most, then prove that every adjacent pair
   is an ancestor relationship. The first member must be based on the default
   branch; every later member must contain the prior member. This produces a
   single order such as `PR-1, PR-2, PR-3`.

   Reject ambiguity rather than guessing: two incomparable ancestors, a
   missing predecessor referenced by a base branch, equal but distinct heads,
   or an intermediate PR that is closed/draft are blockers. Report the PR URLs
   and the observed graph to the user.

4. Record the ordered PR numbers, URLs, head branches, head SHAs, and the
   local coordinator branch for each member. Refresh its pull ref immediately
   before its handoff so the coordinator has the head that the sub-agent will
   merge.

## Merge Loop

Process the verified order one PR at a time. Do not start a later PR until the
previous PR is confirmed merged.

### 1. Delegate the Current PR

Spawn one sub-agent for the current PR and explicitly instruct it to use the
[pr-merge](../pr-merge/SKILL.md) skill for that PR number. Give it this scope:

- merge only this named PR through its complete CI and review workflow;
- use an isolated `./.tmp/` worktree if it must inspect, test, commit, or push
  a repair, so it does not alter the coordinator worktree;
- do not merge, rebase, or push any other PR in the chain; and
- return the PR URL, final pre-merge head SHA, merged SHA, and confirmation
  that GitHub reports `MERGED`.

Wait for the sub-agent's result. If it reports a non-actionable blocker, stop
the chain and surface that evidence. Do not silently skip the PR.

### 2. Preserve the Merged PR History Locally

After GitHub confirms PR-X is merged, refresh the local copy of its final PR
head _before relying on normal remote branch refs_. `refs/pull/<number>/head`
remains available even if GitHub deleted `origin/<head-branch>`:

```bash
git -C <worktree> fetch origin \
  +refs/pull/<PR-X>/head:refs/heads/pr-merge-chain/<top>/<PR-X>
git -C <worktree> fetch --prune origin <default-branch>
git -C <worktree> checkout pr-merge-chain/<top>/<PR-X>
git -C <worktree> merge --no-edit origin/<default-branch>
```

Do **not** push this merge into the already-merged PR-X branch. It reconnects
the original PR commits with the squash commit on the default branch only in
the local coordinator history. If this merge conflicts, abort that merge and
stop with the conflict details; it should normally be a zero-content-change
merge.

### 3. Reconnect and Update the Next PR

If PR-X has a successor PR-(X+1), merge the reconnected local PR-X branch into
the successor's coordinator branch, then push the successor normally:

```bash
git -C <worktree> checkout pr-merge-chain/<top>/<PR-(X+1)>
git -C <worktree> merge --no-edit pr-merge-chain/<top>/<PR-X>
git -C <worktree> push origin \
  HEAD:refs/heads/<PR-(X+1)-head-branch>
```

This is the one required push between chain members. It updates the open
successor PR with a merge commit that makes Git recognize the already-squashed
predecessor content, and it should be conflict-free. Never force-push. If the
push is rejected because someone updated the remote branch, stop and
re-discover the chain from the latest remote state. If it succeeds, record the
new successor head SHA and let its `pr-merge` sub-agent handle the resulting
checks and reviews on the next loop iteration.

For the final PR, no successor update is needed: its sub-agent's confirmed
merge completes the chain.

## Completion and Cleanup

Confirm every recorded PR is `MERGED` with `gh pr view <number> --json state`
and fetch `origin/<default-branch>` one final time. Report the discovered
order, each PR URL and merge SHA, the successor branch updates made, and any
checks or review remediation performed by the sub-agents.

Remove only the dedicated clean coordinator worktree and its private
`pr-merge-chain/...` branches. Leave user branches, unrelated worktrees, and
Git configuration untouched. If cleanup cannot be performed safely, leave the
private worktree in place and report its path.

## Failure Handling

| Situation                             | Action                                                         |
| ------------------------------------- | -------------------------------------------------------------- |
| Top input is not an open PR           | Stop and report its current state.                             |
| Chain is not linear                   | Report the dependency graph; require user-selected order.      |
| A member is draft, closed, or missing | Stop before any merge.                                         |
| A reconciliation merge conflicts      | Abort only that in-progress merge and report the conflict.     |
| Successor push is non-fast-forward    | Do not force-push; re-discover the chain after user direction. |
| A delegated `pr-merge` is blocked     | Stop the chain at that PR and return its evidence.             |

## Related Skills

- [pr-merge](../pr-merge/SKILL.md) — required workflow for each individual
  GitHub PR merge.
- [pr-feedback-resolution](../pr-feedback-resolution/SKILL.md) — remediation
  workflow used by `pr-merge` when CI or review feedback needs a repair.
