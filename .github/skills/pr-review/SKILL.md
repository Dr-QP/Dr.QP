---
name: pr-review
description: 'Perform a thorough automated code review of a GitHub pull request in this ROS 2 robotics workspace, publishing feedback as a single GitHub pull request review with inline comments (never a standalone top-level comment). Use when asked to review a pull request, or when a PR is opened/reopened and an automated review is required. Keywords: PR review, code review, pull request review, automated review.'
---

# PR Review

Use this skill to review a pull request's diff and publish feedback as a GitHub **pull request review** — not a plain issue/PR comment.

## When to Use This Skill

- A pull request was just opened or reopened and needs an automated review
- Someone explicitly asks for a review of a pull request (e.g. "@claude review this PR")

## Prerequisites

- `gh` must be installed and authenticated
- When running locally, mcp tools are `mcp__gateway__*` instead of `mcp__github__*`
- The `mcp__github__pull_request_review_write` and `mcp__github__add_comment_to_pending_review` MCP tools are the primary way to create the review and its inline comments. Check with ToolSearch first; if they are not available in the session, use the single fallback flow described after Step 8 instead. **Never construct `gh api` review-posting calls by hand** (ad-hoc `jq`/heredoc JSON assembly routinely trips Claude Code's Bash safety checks — "brace with quote character", "shell syntax that cannot be statically analyzed", etc. — burning many turns for no result).
- REPO (`owner/name`) and PR NUMBER must be known — read them from the workflow context or ask the user if not provided

## Review Focus

Two independent concerns feed two different pass types (see Steps below) so they can be reviewed without one blind spot masking the other.

**Compliance focus** (repo-convention adherence) — runs on **Sonnet**:

- ROS 2 conventions (node lifecycle, topic/service naming, parameter handling)
- C++/Python style and idioms
- Any rule in the `.github/instructions/*.instructions.md` files applicable to the changed files (see Step 2), and any repo `CLAUDE.md`/`AGENTS.md` file that shares a path with the changed file or its parents
- IGNORE import ordering, that is handled by `ruff` and `clang-format` in CI
- Only flag a violation if you can quote the exact rule text being broken

**Correctness focus** (bug-hunting) — runs on **Opus**:

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

## Steps

1. **Gate.** Run `gh pr view <PR_NUMBER>`. If the PR is a draft, already closed/merged, or trivial (docs-only, version bump, generated-file-only diff), stop here and say so instead of proceeding — do not fetch the diff or spawn any review passes.
2. **Gather context.** Fetch the diff (`gh pr diff <PR_NUMBER>`) and reuse the PR description from Step 1. From the changed-file list, determine which `.github/instructions/*.instructions.md` files apply, by matching each file's `applyTo` frontmatter glob against the changed paths (`engineering.instructions.md`'s `**` always applies).
3. **Run four independent initial-review passes in parallel** — each pass sees only the diff, the PR title, the PR description, and its own focus list; none sees another pass's output. Each pass returns a list of issues, where each issue has a description and the reason it was flagged (e.g. "CLAUDE.md adherence", "bug"):
   - 2x **compliance pass** (Sonnet) — audit the diff against the Compliance focus list and the instruction files found in Step 2.
   - 2x **correctness pass** (Opus) — audit the diff against the Correctness focus list, one pass scanning for obvious bugs and the other for security/logic issues introduced by the changed code.
   - In Claude Code, issue four `Agent` tool calls in a single message (`subagent_type: general-purpose`, `model: sonnet` for the compliance pair, `model: opus` for the correctness pair). In environments without sub-agent spawning, perform the four passes sequentially in this session, each one starting fresh from the diff with no memory of the prior passes, to preserve independence.
   - **Block until every pass reports back or hard-times-out — see "Waiting on parallel passes" below.** Do not proceed to Step 4 with a pass still outstanding.
4. **Merge and deduplicate.** Collect the candidate findings from all passes that completed (see fallback below if any didn't). Collapse candidates that name the same file/line and describe the same underlying issue into one.
5. **Validate each candidate independently, in parallel** — for every surviving candidate, run one validation pass that sees only that single candidate plus the diff/description (not the other candidates, not which pass raised it), and must confirm with high confidence that it is a real, worth-flagging issue. Drop any candidate the validator cannot confirm with high confidence. Use the same Claude-Code-vs-sequential-fallback approach as Step 3, and the same blocking policy in "Waiting on parallel passes" below (cap: one `TaskOutput` call, up to 5 minutes per candidate; a validator that doesn't return in time counts as "cannot confirm" — drop the candidate).
6. Create a pending review with `mcp__github__pull_request_review_write` (`method: "create"`, no `event`).
7. For every validated finding, attach it as an inline comment on the exact file/line with `mcp__github__add_comment_to_pending_review` — this is the only place finding text goes; never describe a finding's location in prose.
8. Submit the review with `mcp__github__pull_request_review_write` (`method: "submit_pending"`), setting `event` to `COMMENT`, `REQUEST_CHANGES`, or `APPROVE` based on severity, and `body` limited to a short overall summary (no per-finding detail — that lives in the inline comments).

### Waiting on parallel passes

Launching the `Agent` calls (Steps 3 and 5) is not enough — you must actually block on them, in-tool, until each one reports back. **Never** end a turn on a text-only status update like "waiting on the remaining passes" and never poll with a no-op `Bash` call (`true`, `sleep 5`, etc.): neither one actually blocks, so the turn can end — and reach `stop_reason: end_turn` — while agents are still running. In a one-shot invocation (e.g. the `claude-code-action` GitHub Action step), nothing is left alive afterward to receive their completion notifications, so those passes, and every step after them (merge, validate, post), silently never happen even though the Action step itself reports success. (This is exactly what happened on PR #414 / run `28765686548`: two of four initial passes were abandoned mid-flight and no review was ever posted.)

Instead:

- After dispatching a batch of `Agent` calls, call `TaskOutput` with `block: true` and an explicit `timeout` (ms) for each task ID that hasn't reported back yet. This genuinely blocks the turn on that task.
- **Budget, so Steps 4–8 still have room inside the Action's 30-minute timeout:** per Step-3 pass, allow up to 2 blocking `TaskOutput` calls of up to 8 minutes each (16-minute hard ceiling). For Step 5 validation passes, allow 1 call of up to 5 minutes each.
- **Hard fallback:** if a pass still hasn't completed when its ceiling is reached, call `TaskStop` on its task ID, drop that pass, and continue with only the passes/validations that did complete — do not block indefinitely on a single hung pass, and do not let one hang stall the whole review. Note in the final review summary body how many of the 4 initial passes completed if any were dropped (a completion-count status line, not a per-finding location reference, so it doesn't conflict with the "never write location references" constraint below).

## Fallback: single-call script

If the MCP tools above are confirmed unavailable (check with ToolSearch first), post the whole review — summary, event, and every inline comment — in one atomic `gh api` call instead of the create/comment/submit dance: GitHub's `POST /repos/{owner}/{repo}/pulls/{pull_number}/reviews` endpoint accepts a `comments[]` array alongside `event` and `body`, so there is no pending-review state to manage or discard.

Write two plain files with the Write tool (never assemble this JSON live in a `Bash` call — that's what trips the safety checks):

- a summary file containing only the short overall review body (no per-finding detail)
- a comments file containing a JSON array of every validated finding: `[{"path": "file.py", "line": 42, "side": "RIGHT", "body": "finding text"}, ...]` (use `[]` or omit the file entirely if no findings survived Step 5)

Then run `scripts/post-review.sh --pr <PR_NUMBER> --event <COMMENT|REQUEST_CHANGES|APPROVE> --summary-file <path> --comments-file <path>` (run with `-h` for full usage; `--repo` defaults to the current repo via `gh repo view`). This single call replaces Steps 6–8 entirely. It is already covered by the `Bash(.github/skills/*/scripts/*)` permission; do not fall further back to typing `gh api` calls by hand.

## Constraints

- **Never post a standalone top-level PR comment** (`gh pr comment`, `mcp__github__add_issue_comment`, etc.) for review findings. All feedback must go through the pull request review flow (steps 6–8) so it renders as a proper review with threaded, resolvable inline comments.
- **Never write location references like "in `file.py` (line 42)" or "around line 10" in the review body or in chat.** Every finding tied to a specific file/line must be an actual inline comment on that file/line via `mcp__github__add_comment_to_pending_review`, not prose pointing at a location.
- Do not submit review text as plain chat/assistant messages.
- Keep each inline comment specific and actionable, scoped to the line(s) it annotates.
- Only findings that survived Step 5 validation may be posted — never publish an unvalidated candidate.
