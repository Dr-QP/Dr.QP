---
name: pr-merge-chain
description: 'Merge a main-targeted chain of GitHub pull requests in dependency order, starting with the ready root and promoting draft successors after their prerequisites merge. Use when a top draft PR includes earlier open PR commits, when asked to merge a PR stack/chain, or to reconcile dependent branches after squash merges. Keywords: merge PR chain, draft PR stack, dependent PRs, main-targeted PRs, squash merge chain, pr-merge-chain.'
license: MIT
---

# Merge a Pull-Request Chain

Merge a linear stack of open pull requests that all target the repository's
default branch. Start with the ready root PR; dependent successors begin as
drafts and list their included predecessor PRs in a `## Dependencies` section.
Delegate each individual GitHub merge to a sub-agent that follows the
[pr-merge](../pr-merge/SKILL.md) skill. Between merges, locally reconnect the
squashed history, push only the repaired successor branch, and mark that
successor ready after all of its declared predecessors are confirmed merged.

## When to Use This Skill

- A draft PR branch includes commits from one or more open predecessor PRs.
- Someone asks to merge a PR stack, chain, or a top PR and all of its
  dependencies.
- A main-targeted dependent PR chain must be reconciled after each squash
  merge and promoted from draft in dependency order.

Do not use this skill for unrelated PRs, a graph with branching dependencies,
or a chain whose members do not all target the repository's default branch.
Report those cases and ask for the intended merge order instead.

## Input and Safety Boundaries

Accept exactly one input: the number, URL, or head branch of the top PR.

- Resolve it with `gh pr view <input>`. It must be open. It may be a draft when
  it declares one or more predecessor PRs; a root input must be ready.
- Require `gh auth status` to succeed and identify the repository default
  branch with `gh repo view --json defaultBranchRef`.
- Use normal local Git commands and `gh` commands. Do not use GitHub API or
  GitHub MCP tools to update refs, and never force-push.
- Preserve the caller's working tree. Coordinate Git history in a dedicated
  temporary worktree under `./.tmp/`; never use `$TMPDIR`.
- Do not begin merging until the complete, linear chain, dependency metadata,
  default-branch bases, and draft states have been verified. The root must be
  ready and every dependent successor must be a draft.

## Discover and Validate the Chain

1. Resolve the top PR with its body, then read only the content below its
   `## Dependencies` heading and before the next level-two heading. The section
   must list every included predecessor PR in merge order, as required by
   `implement-agent-specs`, followed conceptually by the top PR. For example,
   a top PR whose section lists `#123` and `#124` declares the linear order
   `#123, #124, <top>`.

   Resolve every listed PR and inspect each member's own dependency section.
   Each member's list must be the exact prefix that precedes it: the root lists
   none, the second member lists only the root, and so on. Reject missing,
   duplicated, out-of-order, closed, or non-linear dependencies. Do not infer
   the chain from PR base branches because every PR must target the default
   branch.

2. List all open PRs, fetch the latest remote refs, and create an isolated
   detached worktree, for example
   `./.tmp/pr-merge-chain-<top-pr-number>`. Fetch every declared PR and every
   other open candidate's `refs/pull/<number>/head` into a dedicated local
   branch in that worktree. Do this before the first merge: GitHub commonly
   deletes a PR's remote head branch once it is merged. Non-declared candidate
   refs are used only for the omitted-ancestor check.

   ```bash
   git fetch --prune origin
   git worktree add --detach ./.tmp/pr-merge-chain-<top> origin/<default-branch>
   git -C ./.tmp/pr-merge-chain-<top> fetch origin \
     +refs/pull/<number>/head:refs/heads/pr-merge-chain/<top>/<number>
   ```

   These private `pr-merge-chain/...` branches are disposable coordinator
   refs. Never overwrite the user's ordinary local branches.

3. List all open PRs with their number, URL, body, head branch, head SHA, base
   branch, and draft state. Verify every declared member targets the repository
   default branch. The root must not be a draft; every later member must be a
   draft. Equal heads belonging to distinct PRs are ambiguous and must be
   rejected.

   Use Git ancestry to corroborate the declared chain and to detect an open PR
   whose commits are inherited but omitted from the dependency section:

   ```bash
   git -C <worktree> merge-base --is-ancestor \
     pr-merge-chain/<top>/<candidate> pr-merge-chain/<top>/<top>
   ```

   An adjacent predecessor head should normally be an ancestor of its
   successor. A declared pair may have drifted after an automated formatter or
   other focused update changed the predecessor branch. Record that drift but
   do not reject the declared linear chain solely for this reason; the
   reconciliation merge after the predecessor lands must incorporate its final
   head. Reject an undeclared open ancestor, unrelated or branching dependency,
   or conflicting dependency metadata rather than guessing.

4. Record the ordered PR numbers, URLs, dependency lists, head branches, head
   SHAs, draft states, and local coordinator branches. Refresh a member's pull
   ref immediately before its handoff so the coordinator has the head that the
   sub-agent will merge.

## Merge Loop

Process the verified order one PR at a time. Do not start a later PR until the
previous PR is confirmed merged.

### 1. Verify or Promote the Current PR

Refresh the current PR's state, base, body, and head SHA before handoff. For
the root PR, confirm it is still open, ready, and based on the default branch.

For a dependent successor, first confirm every PR in its declared dependency
list is `MERGED`, its base is still the default branch, its dependency section
is unchanged, and its head matches the coordinator SHA pushed by the previous
iteration. It must still be a draft at this point. Mark it ready, then verify
that GitHub reports `isDraft: false`:

```bash
gh pr view <current> \
  --json state,isDraft,baseRefName,body,headRefOid
gh pr ready <current>
gh pr view <current> --json state,isDraft,baseRefName,headRefOid
```

Do not remove the dependency section when promoting the PR; it remains useful
history explaining the included commits and merge order. Do not promote a
later successor early, and never hand a draft PR to `pr-merge`.

### 2. Delegate the Current PR

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

### 3. Preserve the Merged PR History Locally

After GitHub confirms PR-X is merged, refresh the local copy of its final PR
head _before relying on normal remote branch refs_. `refs/pull/<number>/head`
remains available even if GitHub deleted `origin/<head-branch>`:

```bash
git -C <worktree> checkout --detach
git -C <worktree> fetch origin \
  +refs/pull/<PR-X>/head:refs/heads/pr-merge-chain/<top>/<PR-X>
git -C <worktree> fetch --prune origin <default-branch>
git -C <worktree> checkout pr-merge-chain/<top>/<PR-X>
git -C <worktree> merge --no-edit origin/<default-branch>
```

Detaching first is required after a previous iteration checked out a
coordinator branch: Git refuses to fetch into a branch that is currently
checked out by any worktree.

Do **not** push this merge into the already-merged PR-X branch. It reconnects
the original PR commits with the squash commit on the default branch only in
the local coordinator history. If this merge conflicts, abort that merge and
stop with the conflict details; it should normally be a zero-content-change
merge.

### 4. Reconnect and Update the Next PR

If PR-X has a successor PR-(X+1), first confirm the successor is still an open
draft based on the default branch. Merge the reconnected local PR-X branch
into the successor's coordinator branch, then push the successor normally:

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
new successor head SHA. Start the next loop iteration, confirm all of that
successor's declared predecessors are merged, and promote it before its
`pr-merge` handoff. Leave every later successor as a draft.

For the final PR, no successor update is needed: its sub-agent's confirmed
merge completes the chain.

## Completion and Cleanup

Confirm every recorded PR is `MERGED` with `gh pr view <number> --json state`
and fetch `origin/<default-branch>` one final time. Report the discovered
order, each PR URL and merge SHA, each draft-to-ready transition, the successor
branch updates made, and any checks or review remediation performed by the
sub-agents.

Remove only the dedicated clean coordinator worktree and its private
`pr-merge-chain/...` branches. Leave user branches, unrelated worktrees, and
Git configuration untouched. If cleanup cannot be performed safely, leave the
private worktree in place and report its path.

## Failure Handling

| Situation                                  | Action                                                         |
| ------------------------------------------ | -------------------------------------------------------------- |
| Top input is not an open PR                | Stop and report its current state.                             |
| Dependency metadata is missing/non-linear  | Report the declared graph; require corrected metadata.         |
| A member does not target the default branch | Stop before any merge and report its base.                    |
| Root is draft or successor is prematurely ready | Stop before any merge and report the invalid state.       |
| A dependency is closed or missing          | Stop before any merge.                                         |
| A reconciliation merge conflicts           | Abort only that in-progress merge and report the conflict.     |
| Successor push is non-fast-forward         | Do not force-push; re-discover the chain after user direction. |
| Successor cannot be promoted               | Keep it draft and report the GitHub error.                     |
| A delegated `pr-merge` is blocked          | Stop the chain at that PR and return its evidence.             |

## Related Skills

- [implement-agent-specs](../implement-agent-specs/SKILL.md) — creates the
  main-targeted ready-root/draft-successor chains consumed by this workflow.
- [pr-merge](../pr-merge/SKILL.md) — required workflow for each individual
  GitHub PR merge.
- [pr-feedback-resolution](../pr-feedback-resolution/SKILL.md) — remediation
  workflow used by `pr-merge` when CI or review feedback needs a repair.
