---
name: pr-review
description: 'Perform a thorough automated code review of a GitHub pull request in this ROS 2 robotics workspace, posting top-level and inline comments via gh and the GitHub inline-comment MCP tool. Use when asked to review a pull request, or when a PR is opened/reopened and an automated review is required. Keywords: PR review, code review, pull request review, automated review.'
---

# PR Review

Use this skill to review a pull request's diff and publish feedback as GitHub comments.

## When to Use This Skill

- A pull request was just opened or reopened and needs an automated review
- Someone explicitly asks for a review of a pull request (e.g. "@claude review this PR")

## Prerequisites

- `gh` must be installed and authenticated
- The `mcp__github_inline_comment__create_inline_comment` tool must be available for inline comments
- REPO (`owner/name`) and PR NUMBER must be known — read them from the workflow context or ask the user if not provided

## Review Focus

Analyze the full diff and provide a thorough code review focused on:

- Code quality, correctness, and best practices
- Potential bugs or edge cases
- Security implications
- Performance considerations
- ROS 2 conventions (node lifecycle, topic/service naming, parameter handling)
- C++/Python style and idioms
- Test coverage and quality

## Steps

1. Fetch the diff: `gh pr diff <PR_NUMBER>`
2. Fetch the PR description for context: `gh pr view <PR_NUMBER>`
3. Analyze the changes against the review focus areas above.
4. Post overall, top-level feedback with `gh pr comment <PR_NUMBER> --body "..."`.
5. Post line-specific feedback with `mcp__github_inline_comment__create_inline_comment` (with `confirmed: true`).

## Constraints

- Only post GitHub comments — do not submit review text as plain chat/assistant messages.
- Keep feedback specific and actionable; cite file paths and line numbers.
