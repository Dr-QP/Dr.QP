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
- The `mcp__github__pull_request_review_write` and `mcp__github__add_comment_to_pending_review` MCP tools are the primary way to create the review and its inline comments. If they are not available in the session (check with ToolSearch first), fall back to the equivalent `gh api` calls — see Fallback below.
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
4. **Merge and deduplicate.** Collect the candidate findings from all four passes. Collapse candidates that name the same file/line and describe the same underlying issue into one.
5. **Validate each candidate independently, in parallel** — for every surviving candidate, run one validation pass that sees only that single candidate plus the diff/description (not the other candidates, not which pass raised it), and must confirm with high confidence that it is a real, worth-flagging issue. Drop any candidate the validator cannot confirm with high confidence. Use the same Claude-Code-vs-sequential-fallback approach as Step 3.
6. Create a pending review with `mcp__github__pull_request_review_write` (`method: "create"`, no `event`). Fallback: `gh api repos/<owner>/<repo>/pulls/<PR_NUMBER>/reviews -f commit_id="$(gh pr view <PR_NUMBER> --json headRefOid -q .headRefOid)"` (omitting `-f event` keeps it pending).
7. For every validated finding, attach it as an inline comment on the exact file/line with `mcp__github__add_comment_to_pending_review` — this is the only place finding text goes; never describe a finding's location in prose. Fallback: look up the pending review's id (`gh api repos/<owner>/<repo>/pulls/<PR_NUMBER>/reviews --jq '.[] | select(.state=="PENDING") | .id'`), then `gh api repos/<owner>/<repo>/pulls/<PR_NUMBER>/comments -f body="<finding>" -f path="<file>" -F line=<n> -f side=RIGHT -F pull_request_review_id=<id>` (repeat per finding).
8. Submit the review with `mcp__github__pull_request_review_write` (`method: "submit_pending"`), setting `event` to `COMMENT`, `REQUEST_CHANGES`, or `APPROVE` based on severity, and `body` limited to a short overall summary (no per-finding detail — that lives in the inline comments). Fallback: `gh api repos/<owner>/<repo>/pulls/<PR_NUMBER>/reviews/<id>/events -f event=<COMMENT|REQUEST_CHANGES|APPROVE> -f body="<summary>"`.

## Fallback: gh api equivalents

Use these only when the MCP tools above are confirmed unavailable in the session — they require the `Bash(gh api repos/*/pulls/*/reviews:*)`, `Bash(gh api repos/*/pulls/*/reviews/*:*)`, and `Bash(gh api repos/*/pulls/*/comments:*)` entries already present in the workflow's `allowedTools`. If a pending review must be discarded instead of submitted (e.g. every candidate failed Step 5 validation after Step 6 already created one), delete it rather than leaving it dangling: `gh api repos/<owner>/<repo>/pulls/<PR_NUMBER>/reviews/<id> -X DELETE`.

## Constraints

- **Never post a standalone top-level PR comment** (`gh pr comment`, `mcp__github__add_issue_comment`, etc.) for review findings. All feedback must go through the pull request review flow (steps 6–8) so it renders as a proper review with threaded, resolvable inline comments.
- **Never write location references like "in `file.py` (line 42)" or "around line 10" in the review body or in chat.** Every finding tied to a specific file/line must be an actual inline comment on that file/line via `mcp__github__add_comment_to_pending_review`, not prose pointing at a location.
- Do not submit review text as plain chat/assistant messages.
- Keep each inline comment specific and actionable, scoped to the line(s) it annotates.
- Only findings that survived Step 5 validation may be posted — never publish an unvalidated candidate.
