---
name: pr-review
description: 'Perform a thorough automated code review of a GitHub pull request in this ROS 2 robotics workspace, publishing feedback as a single GitHub pull request review with inline comments (never a standalone top-level comment). Use when asked to review a pull request, or when a PR is opened/reopened and an automated review is required. Keywords: PR review, code review, pull request review, automated review.'
---

# PR Review

Use this skill to review a pull request's diff and publish feedback as a GitHub **pull request review** — not a plain issue/PR comment. The workflow is agent-agnostic: Codex, GitHub Copilot coding agents, Claude Code, and other assistants should follow the same review policy while adapting tool names to the environment they are running in.

## When to Use This Skill

- A pull request was just opened or reopened and needs an automated review
- Someone explicitly asks for a review of a pull request (for example, "@codex review this PR", "@copilot review this PR", or "@claude review this PR")

## Prerequisites

- `gh` must be installed and authenticated
- REPO (`owner/name`) and PR NUMBER must be known — read them from the workflow context or ask the user if not provided
- The review is created, annotated, and submitted with three separate GitHub review tools:
  - `create_pending_pull_request_review` — open a pending review
  - `add_comment_to_pending_review` — attach each inline comment to the pending review
  - `submit_pending_pull_request_review` — submit the pending review with an `event`
- Codex-specific option: if the GitHub connector is available use
  `mcp__codex_apps__github._add_review_to_pr`, use that tool to submit the review in one call with:
  - `action`: `APPROVE`, `COMMENT`, or `REQUEST_CHANGES`
  - `review`: the short overall summary body
  - `file_comments`: every validated inline finding, using `path`, `line`, `side`, and `body`

  This still creates a proper pull request review with inline comments, not a standalone PR comment.

- If GitHub rejects `REQUEST_CHANGES` because the authenticated account owns the PR, retry with `COMMENT` and state in the review body that GitHub would not allow that account to request changes on its own pull request.
- If those review tools are unavailable or uncertain, use the single-call fallback flow described after Step 8. **Never construct `gh api` review-posting calls by hand**.

## Review Focus

Two independent concerns feed two different pass types (see Steps below) so they can be reviewed without one blind spot masking the other.

**Compliance focus** (repo-convention adherence) — use a fast, instruction-following reviewer:

- ROS 2 conventions (node lifecycle, topic/service naming, parameter handling)
- C++/Python style and idioms
- Any rule in the `.github/instructions/*.instructions.md` files applicable to the changed files (see Step 2), and any repo `CLAUDE.md`/`AGENTS.md` file that shares a path with the changed file or its parents
- IGNORE import ordering, that is handled by `ruff` and `clang-format` in CI
- Only flag a violation if you can quote the exact rule text being broken

**Correctness focus** (bug-hunting) — use the strongest available reasoning reviewer:

- Scan only the diff itself, without pulling in extra context beyond the diff and the PR title/description — do not flag anything you cannot validate from the diff alone
- Potential bugs, incorrect logic, and security implications introduced by the changed code
- Test coverage and quality
- Flag only significant, high-confidence issues; ignore nitpicks and likely false positives

**CRITICAL: we only want HIGH SIGNAL issues.** Flag an issue only when at least one of these holds:

- The code will fail to compile or parse (syntax errors, type errors, missing imports, unresolved references)
- The code will definitely produce wrong results regardless of inputs (clear, unambiguous logic errors)
- It's a clear, unambiguous compliance violation where you can quote the exact rule being broken

Do NOT flag:

- Code style or quality concerns
- Potential issues that depend on specific inputs or state
- Subjective suggestions or improvements

Flag only significant bugs; ignore nitpicks and likely false positives. Do not flag issues that you cannot validate without looking at context outside of the git diff.

**Severity tiers** (carried through Steps 3–8 on every candidate/validated finding):

- **Blocking (critical/P1)** — anything from a **correctness pass**: compile/parse failures, definite-wrong-result logic bugs, security implications.
- **Non-blocking** — anything from a **compliance pass**: repo-convention/style violations, even though they're quoted-rule-confirmed.

## Steps

1. **Gate.** Run `gh pr view <PR_NUMBER>`. If the PR is a draft, already closed/merged, or trivial (docs-only, version bump, generated-file-only diff), stop here and say so instead of proceeding — do not fetch the diff or spawn any review passes.
2. **Gather context.** Fetch the diff (`gh pr diff <PR_NUMBER>`) and reuse the PR description from Step 1. From the changed-file list, determine which `.github/instructions/*.instructions.md` files apply, by matching each file's `applyTo` frontmatter glob against the changed paths (`engineering.instructions.md`'s `**` always applies).
3. **Run four independent initial-review passes in parallel when the environment supports it** — each pass sees only the diff, the PR title, the PR description, and its own focus list; none sees another pass's output. Each pass returns a list of issues, where each issue has a description and the reason it was flagged (for example, "AGENTS.md adherence", "bug", or "security"):
   - 2x **compliance pass** — audit the diff against the Compliance focus list and the instruction files found in Step 2.
   - 2x **correctness pass** — audit the diff against the Correctness focus list, one pass scanning for obvious bugs and the other for security/logic issues introduced by the changed code.
   - Codex: use available multi-agent/sub-agent tools when present; otherwise perform the four passes sequentially in this session, restarting the review lens from the diff for each pass.
   - GitHub Copilot coding agent: if no sub-agent facility is available, perform the four passes sequentially in this session, explicitly clearing prior pass conclusions before starting the next pass.
   - Claude Code: issue four `Agent` tool calls in a single message (`subagent_type: general-purpose`; use a fast model for compliance and the strongest available model for correctness).
   - **Block until every pass reports back or hard-times-out — see "Waiting on parallel passes" below.** Do not proceed to Step 4 with a pass still outstanding.
4. **Merge and deduplicate.** Collect the candidate findings from all passes that completed (see fallback below if any didn't). Collapse candidates that name the same file/line and describe the same underlying issue into one, keeping the **blocking** tier if either collapsed candidate was blocking.
5. **Validate each candidate independently, in parallel when supported** — for every surviving candidate, run one validation pass that sees only that single candidate plus the diff/description (not the other candidates, not which pass raised it), and must confirm with high confidence that it is a real, worth-flagging issue. Drop any candidate the validator cannot confirm with high confidence. Preserve each surviving candidate's severity tier from Step 4 unchanged — validation confirms or drops a finding, it never changes its tier. Use the same runner-specific parallel-vs-sequential approach as Step 3, and the same blocking policy in "Waiting on parallel passes" below (cap: one blocking wait, up to 5 minutes per candidate; a validator that doesn't return in time counts as "cannot confirm" — drop the candidate).
6. Create a pending review with `create_pending_pull_request_review`.
7. For every validated finding, attach it as an inline comment on the exact file/line with `add_comment_to_pending_review` — this is the only place finding text goes; never describe a finding's location in prose.
8. Submit the review with `submit_pending_pull_request_review`, choosing `event` from the validated findings that survived Step 5:
   - **No validated findings at all → `APPROVE`.** A clean pass must be approved, not left as a silent `COMMENT`.
   - **At least one `blocking` (critical/P1) finding → `REQUEST_CHANGES`.**
   - **Only non-blocking findings, none blocking → `COMMENT`.**

   `body` is limited to a short overall summary (no per-finding detail — that lives in the inline comments); for an `APPROVE` with zero findings, state plainly that no issues were found.

   Codex: if using `mcp__codex_apps__github._add_review_to_pr` instead of Steps 6-8, pass the same event as `action`,
   the summary as `review`, and all validated inline findings as `file_comments`. If `REQUEST_CHANGES` fails because
   GitHub disallows requesting changes on the authenticated user's own PR, retry the same inline review with
   `action: "COMMENT"` and include a brief note in `review` explaining the event downgrade.

### Waiting on Parallel Passes

Launching parallel review workers (Steps 3 and 5) is not enough — you must actually block on them, in-tool, until each one reports back. **Never** end a turn on a text-only status update like "waiting on the remaining passes" and never poll with a no-op shell command (`true`, `sleep 5`, etc.): neither one actually blocks a one-shot agent invocation, so the remaining merge, validate, and publish steps may never run.

Instead:

- After dispatching a batch of workers, use the runner's blocking wait primitive for each task ID that has not reported back yet:
  - Codex: use the multi-agent tool's blocking output/wait facility if available.
  - GitHub Copilot coding agent: if no blocking worker primitive exists, do not launch background work; run the passes sequentially.
  - Claude Code: call `TaskOutput` with `block: true` and an explicit `timeout` (ms) for each outstanding task ID.
- **Budget, so Steps 4–8 still have room inside the action timeout:** per Step-3 pass, allow up to 16 minutes total. For Step 5 validation passes, allow up to 5 minutes each.
- **Hard fallback:** if a pass still has not completed when its ceiling is reached, stop/cancel it if the runner supports cancellation, drop that pass, and continue with only the passes/validations that did complete — do not block indefinitely on a single hung pass, and do not let one hang stall the whole review. Note in the final review summary body how many of the 4 initial passes completed if any were dropped (a completion-count status line, not a per-finding location reference, so it does not conflict with the "never write location references" constraint below).

## Fallback: single-call script

If the review tools above are confirmed unavailable, post the whole review — summary, event, and every inline comment — through the repository script in one atomic GitHub review API call. GitHub's `POST /repos/{owner}/{repo}/pulls/{pull_number}/reviews` endpoint accepts a `comments[]` array alongside `event` and `body`, so there is no pending-review state to manage or discard.

Write two plain files under `./.tmp` (never assemble this JSON live in a shell command):

- a summary file containing only the short overall review body (no per-finding detail)
- a comments file containing a JSON array of every validated finding: `[{"path": "file.py", "line": 42, "side": "RIGHT", "body": "finding text"}, ...]` (use `[]` or omit the file entirely if no findings survived Step 5)

Then run `scripts/post-review.sh --pr <PR_NUMBER> --event <COMMENT|REQUEST_CHANGES|APPROVE> --summary-file <path> --comments-file <path>` (run with `-h` for full usage; `--repo` defaults to the current repo via `gh repo view`). This single call replaces Steps 6–8 entirely, including the `event` decision rule from Step 8 (`APPROVE` when `[]`/no findings, `REQUEST_CHANGES` when any surviving finding is blocking, `COMMENT` otherwise). Do not fall further back to typing `gh api` calls by hand.

## Constraints

- **Never post a standalone top-level PR comment** (`gh pr comment`, `add_issue_comment`, etc.) for review findings. All feedback must go through the pull request review flow (steps 6–8) so it renders as a proper review with threaded, resolvable inline comments.
- **Never write location references like "in `file.py` (line 42)" or "around line 10" in the review body or in chat.** Every finding tied to a specific file/line must be an actual inline comment on that file/line via `add_comment_to_pending_review`, not prose pointing at a location.
- Do not submit review text as plain chat/assistant messages.
- Keep each inline comment specific and actionable, scoped to the line(s) it annotates.
- Only findings that survived Step 5 validation may be posted — never publish an unvalidated candidate.
