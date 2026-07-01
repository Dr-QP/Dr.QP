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
- The `mcp__gateway__pull_request_review_write` and `mcp__gateway__add_comment_to_pending_review` MCP tools must be available for creating the review and its inline comments
- REPO (`owner/name`) and PR NUMBER must be known — read them from the workflow context or ask the user if not provided

## Review Focus

Analyze the full diff and provide a thorough code review focused on:

- Code quality, correctness, and best practices
- Potential bugs or edge cases
- Security implications
- Performance considerations
- ROS 2 conventions (node lifecycle, topic/service naming, parameter handling)
- C++/Python style and idioms
- IGNORE import ordering, that is handled by `ruff` and `clang-format` in CI
- Test coverage and quality

## Steps

1. Fetch the diff: `gh pr diff <PR_NUMBER>`
2. Fetch the PR description for context: `gh pr view <PR_NUMBER>`
3. Analyze the changes against the review focus areas above.
4. Create a pending review with `mcp__gateway__pull_request_review_write` (`method: "create"`, no `event`).
5. For every finding, attach it as an inline comment on the exact file/line with `mcp__gateway__add_comment_to_pending_review` — this is the only place finding text goes; never describe a finding's location in prose.
6. Submit the review with `mcp__gateway__pull_request_review_write` (`method: "submit_pending"`), setting `event` to `COMMENT`, `REQUEST_CHANGES`, or `APPROVE` based on severity, and `body` limited to a short overall summary (no per-finding detail — that lives in the inline comments).

## Constraints

- **Never post a standalone top-level PR comment** (`gh pr comment`, `mcp__gateway__add_issue_comment`, etc.) for review findings. All feedback must go through the pull request review flow (steps 4–6) so it renders as a proper review with threaded, resolvable inline comments.
- **Never write location references like "in `file.py` (line 42)" or "around line 10" in the review body or in chat.** Every finding tied to a specific file/line must be an actual inline comment on that file/line via `mcp__gateway__add_comment_to_pending_review`, not prose pointing at a location.
- Do not submit review text as plain chat/assistant messages.
- Keep each inline comment specific and actionable, scoped to the line(s) it annotates.
