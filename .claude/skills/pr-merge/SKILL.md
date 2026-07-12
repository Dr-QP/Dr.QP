---
name: pr-merge
description: 'Merge an open GitHub pull request: leave existing auto-merge unchanged, otherwise enable automatic squash merge early, monitor CI and AI review, remediate failures and feedback, then squash merge explicitly if auto-merge is unavailable. Use when a PR needs to be merged or shepherded through CI and review to merge. Keywords: merge PR, auto-merge, squash merge, monitor PR, merge-ready, wait for CI, CI failures, AI review, Claude Responder.'
license: MIT
---

# Merge PR

Drive one open pull request through the complete CI-and-review cycle and merge
it. At the start, leave any existing auto-merge unchanged; otherwise enable
automatic squash merge. If auto-merge is unavailable, merge explicitly after
all requirements pass. This is a persistent workflow: wait for the relevant
GitHub state to change, then act on the new state.

## When to Use This Skill

- A pull request has just been opened and must be merged after CI and review.
- Someone asks to merge, babysit, or shepherd a PR through CI and review.
- CI is still running, an automated AI review is pending, or review feedback
  must be addressed before merging.

Do not use this skill for a draft, closed, or merged PR unless the user first
asks to change that state.

## Prerequisites and Scope

- Resolve the PR from an explicit number or URL, or use the current branch:

  ```bash
  gh pr view --json number,url,headRefName,headRefOid,isDraft,state
  ```

- Verify GitHub CLI authentication before monitoring:

  ```bash
  gh auth status
  ```

- Work from an isolated worktree. If the caller did not provide one, create
  one under `./.tmp/` after resolving the PR number and head branch:

  ```bash
  mkdir -p ./.tmp
  git fetch origin <head-branch>
  git worktree add --detach ./.tmp/pr-merge-<pr> origin/<head-branch>
  git -C ./.tmp/pr-merge-<pr> switch -c pr-merge-worktree/<pr>
  ```

  Run all local inspection, tests, commits, and pushes from that worktree.
  Push its `HEAD` only to the resolved PR head branch; never push its private
  `pr-merge-worktree/<pr>` name. Preserve the caller's worktree and unrelated
  local branches. After a confirmed merge, remove only the clean worktree and
  private branch that this skill created; otherwise leave them in place and
  report their path and state.

- Record the initial head SHA, CI check names and URLs, AI-review state, and
  unresolved review threads. Keep this state in the conversation; do not post
  progress comments to the PR.
- Work only on the PR head branch. Preserve unrelated local changes, never
  force-push, and never update its base branch unless the user explicitly
  requests it. This skill's merge workflow authorizes enabling auto-merge and,
  when needed, performing the final squash merge.
- A remediation requested by this skill is authorized by the user request to
  make the PR mergeable. Still stop for a genuinely ambiguous review request
  and ask for clarification in that review thread.

## Reformat Workflow Synchronization

`reformat.yml` can push a formatting commit to the PR branch as its job
finishes. Treat its completion as a possible head change, not as proof that
the checks and reviews already observed apply to the final branch state.

Track every `reformat.yml` run for the current PR head. After such a run
reaches `completed`, immediately refresh the PR head and fast-forward the
isolated worktree:

```bash
gh run list --workflow=reformat.yml --branch <head-branch> \
  --json databaseId,status,conclusion,headSha,updatedAt,url --limit 20
gh pr view <pr> --json headRefName,headRefOid
git -C <worktree> fetch origin <head-branch>
git -C <worktree> merge --ff-only origin/<head-branch>
```

The fetch plus fast-forward merge is the required local pull of any formatter
commit. If the PR head SHA changed, record the new SHA and restart the
monitoring loop from step 1: CI, AI review, feedback, and merge state must all
be re-evaluated for that new head. If the fast-forward fails, do not create an
automatic merge; report the local divergence and resolve it only through the
normal focused-remediation workflow.

## Configure Merge Once

Immediately after resolving the PR and confirming it is open and not a draft,
inspect its automatic merge request:

```bash
gh pr view <pr> --json autoMergeRequest
```

If `autoMergeRequest` is non-null, leave it unchanged and continue monitoring.
Otherwise, enable automatic squash merge before waiting for CI or review:

```bash
gh pr merge <pr> --auto --squash
```

If GitHub reports auto-merge is unavailable or disabled for the repository,
continue the monitoring loop and use the explicit final squash merge below.
Treat any other error as a concrete blocker to investigate or report.

## Completion Criteria

Treat the PR as complete only when all of these are true for its current head
branch:

- It is open and not a draft.
- Every reported CI check has completed successfully. Report external checks
  that cannot be inspected, but do not claim they passed without evidence or
  call the PR merge-ready while one is failing or pending.
- The `Require AI Review` gate is successful and an accepted Claude or Codex
  review exists for the PR's current review cycle.
- The review agent has finished; all of its actionable, unresolved feedback is
  fixed, replied to, and resolved using the feedback-resolution workflow.
- `gh pr view` reports no remaining merge blocker (for example, a conflict or
  missing required approval) before the merge is attempted.
- GitHub reports the PR as merged.

## Monitoring Loop

Repeat this loop until the completion criteria are met, the PR becomes closed
or draft, or a non-actionable blocker needs user direction. Use a blocking
watch or repeated bounded waits; never replace a wait with a text-only promise
to check later.

1. Refresh the PR and capture its head SHA and merge state:

   ```bash
   gh pr view <pr> --json number,url,state,isDraft,headRefName,headRefOid,mergeStateStatus,reviewDecision,reviews
   gh pr checks <pr> --json name,state,bucket,link,workflow,startedAt,completedAt
   ```

2. If checks are pending or queued, wait for GitHub to finish them, then
   refresh both commands above. Prefer:

   ```bash
   gh pr checks <pr> --watch --interval 30
   ```

   If the execution environment limits a long-running command, run bounded
   waits and immediately continue the loop. A `--watch` non-zero exit caused
   by a failed check is a state change to investigate, not a reason to stop.
   When a `reformat.yml` run associated with the observed head has completed,
   perform **Reformat Workflow Synchronization** before classifying checks. If
   it pushed a formatter commit, restart at step 1 and do not act on the stale
   checks or reviews.

3. Classify every completed non-success check before changing code:
   - A GitHub Actions failure: use
     [extract-github-actions-logs](../extract-github-actions-logs/SKILL.md)
     to collect the failing job log and artifacts, then use
     [pr-feedback-resolution](../pr-feedback-resolution/SKILL.md) to diagnose,
     implement, test, commit, and push the focused repair.
   - A CodeQL or Codecov failure: use the same feedback-resolution skill and
     its linked security or coverage workflow.
   - An external check: record its URL and report it as an external blocker;
     do not fabricate a GitHub Actions diagnosis.
   - `Require AI Review` / `ai-review-present`: follow **AI Review Recovery**
     below. Do not try to repair this gate with a code change.
   - A cancelled, timed-out, or infrastructure failure: rerun only when the
     repository permits it and the failure is demonstrably transient;
     otherwise report it as a blocker with its evidence.

4. After a code change, run the narrowest relevant local verification. For ROS
   packages, use `scripts/with-ros-env.sh` and a package-specific selector.
   Commit and push through normal local Git workflow, then restart the loop
   from step 1 because the head SHA and checks have changed.

5. Once CI is successful, inspect reviews and unresolved review threads. Wait
   for an in-progress review agent before declaring success. Apply
   [pr-feedback-resolution](../pr-feedback-resolution/SKILL.md) once to gather
   and address all actionable feedback together, rather than handling comments
   piecemeal. Its resolved-thread filtering and confidence rules are mandatory.

6. If feedback leads to a push, restart at step 1. If no actionable feedback
   remains, refresh checks and merge state once more. Perform **Reformat
   Workflow Synchronization** after that final refresh as well. If it changes
   the head SHA, restart at step 1. Otherwise, if auto-merge was set, wait for
   GitHub to merge the PR; if it was not, perform the explicit final squash
   merge.

## Explicit Final Squash Merge

Use this section only when the initial auto-merge check was empty and the early
`--auto --squash` request was unavailable. After every pre-merge completion
criterion has been evidenced for the current head SHA, merge the PR explicitly:

```bash
gh pr merge <pr> --squash
```

Refresh the PR afterwards and confirm that its state is `MERGED`. If GitHub
rejects the merge because the state changed, return to the monitoring loop;
otherwise report the concrete error as a blocker.

## AI Review Recovery

This repository's `Require AI Review` check accepts a submitted review from
Claude or Codex. Its `Claude Responder` workflow runs a pull-request review
when an issue comment contains `@claude review`.

1. Confirm that the failed AI-review check actually says no accepted AI review
   was found. Inspect its log before triggering anything.
2. Check current PR reviews and active `Claude Responder` runs first. If a
   responder is already reviewing this PR, wait for that run rather than
   starting a duplicate review. Maintain a per-head-SHA trigger ledger in the
   conversation so a trigger is posted at most once for each head SHA.

   **`gh pr checks <pr>` is not sufficient to find these runs.**
   `claude-review.yml` triggers on `issue_comment`, `pull_request_review`,
   and `pull_request_review_comment` in addition to `pull_request`. For those
   non-`pull_request` events, GitHub records the run against
   `context.sha`/`context.ref` — the repo's default branch — not the PR's
   head SHA, even though the workflow's "Determine checkout ref" step then
   checks out the PR's real head commit internally. Because `gh pr checks`
   only lists checks attached to the PR head commit, a `Claude Responder` run
   started by an `@claude review` comment routinely never appears there,
   including while it is still running. Always cross-check the workflow's
   own run list instead of trusting `gh pr checks` alone:

   ```bash
   # All recent Claude Responder runs, regardless of which branch/SHA GitHub filed them under
   gh run list --workflow=claude-review.yml \
     --json databaseId,status,conclusion,event,headBranch,headSha,createdAt,url,displayTitle \
     --limit 20

   # Narrow to in-flight runs only
   gh run list --workflow=claude-review.yml --status in_progress \
     --json databaseId,event,createdAt,url,displayTitle
   gh run list --workflow=claude-review.yml --status queued \
     --json databaseId,event,createdAt,url,displayTitle

   # Narrow to runs from comment triggers specifically (these are the ones gh pr checks misses)
   gh run list --workflow=claude-review.yml --event issue_comment \
     --json databaseId,status,conclusion,createdAt,url,displayTitle --limit 10
   ```

   To confirm a specific run actually belongs to this PR (its `headBranch`
   will usually read as the default branch, not the PR branch, for
   comment-triggered runs), open the run and check the "Determine checkout
   ref" step output or the PR number embedded in its logs:

   ```bash
   gh run view <run-id> --json databaseId,status,conclusion,event,headBranch,headSha,url
   gh run view <run-id> --log | grep -A3 'Determine checkout ref'
   ```

   If an in-progress or queued run is confirmed to be reviewing this PR's
   current head SHA, wait on it directly instead of posting a new trigger:

   ```bash
   gh run watch <run-id> --exit-status
   ```

3. If no review is in progress and no trigger has been posted for that head
   SHA, post exactly this comment:

   ```bash
   gh pr comment <pr> --body '@claude review'
   ```

   This is an operational trigger, not a review finding or status update.
   Immediately after posting it, capture the new run so you can watch it
   directly instead of polling `gh pr checks`:

   ```bash
   gh run list --workflow=claude-review.yml --event issue_comment \
     --json databaseId,status,createdAt,url --limit 1
   ```

4. Wait for the `Claude Responder` workflow to complete — via `gh run watch
<run-id> --exit-status` on the run captured above, or by repeating the
   `gh run list --workflow=claude-review.yml` query until its `status`
   reaches `completed` — then refresh both `gh pr checks` and PR reviews. Do
   not treat workflow start, a plain comment, or a pending review as
   completion.
5. If the responder publishes actionable feedback, return to the review step
   in the monitoring loop and use `pr-feedback-resolution` to address all of
   it. If it publishes a clean review, wait for the `Require AI Review` gate
   to rerun and pass.
6. If the responder finishes without a submitted review, inspect its logs and
   report that concrete failure. Do not repeatedly post the trigger comment;
   require a new head SHA or user direction before retrying.

## Final Report

Report the PR URL, merged SHA, merge method, final check status, AI-review
outcome, feedback resolved, local verification run, and any remaining external
or policy blocker. Say the PR was merged only after its merged state is
evidenced.

## Related Skills

- [pr-feedback-resolution](../pr-feedback-resolution/SKILL.md) — collect and
  resolve all review feedback and actionable CI findings in one pass.
- [extract-github-actions-logs](../extract-github-actions-logs/SKILL.md) —
  retrieve GitHub Actions logs and ROS test artifacts.
- [pr-review](../pr-review/SKILL.md) — automated-review behavior used by the
  Claude Responder workflow.
