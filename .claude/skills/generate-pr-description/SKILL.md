---
name: generate-pr-description
description: Generate comprehensive pull request description following code-review-standards with change analysis, testing strategy, and migration notes. Use when creating a PR, writing PR description, preparing for code review, or documenting technical decisions.
---

# Generate Pull Request Description

Generate a comprehensive PR description by analyzing the change set and filling the repository template at [../../../.github/pull_request_template.md](../../../.github/pull_request_template.md). Use [code-review-standards](../code-review-standards/) for wording and review conventions, and use the Coding Conventions section of the repository [AGENTS.md](../../../AGENTS.md) only for shared quality expectations rather than repeating them here.

## When to Use This Skill

- Creating a pull request for code changes
- Need detailed PR description that explains changes
- Want to follow project conventions for PR documentation
- Preparing for code review
- Documenting technical decisions and assumptions

## Prerequisites

- Changes committed to a branch
- Understanding of what was changed and why
- Related GitHub issues identified (optional)
- Testing performed

## Inputs

- **Base Ref** or **Commit Range**: Default to the PR merge base with
  `origin/main`; accept an explicit range when the caller provides one.
- **Related Issues**: GitHub issue numbers
- **Breaking Changes**: Yes/No
- **Migration Steps**: If breaking

## Workflow

### Step 1: Analyze Git Changes

```bash
base_ref=origin/main
merge_base="$(git merge-base "$base_ref" HEAD)"
git diff --name-status "$merge_base"..HEAD
git diff "$merge_base"..HEAD
git log --oneline "$merge_base"..HEAD
git diff --stat "$merge_base"..HEAD
```

Use the merge base rather than a raw `origin/main..HEAD` range when the branch
and its base have diverged: PR review compares the changes introduced by the
head branch since that common ancestor. If the caller supplies an explicit
commit range or the PR targets a different base branch, use that scope instead.

Categorize: new, modified, deleted, renamed.

### Step 2: Identify Change Categories

Features, Bug Fixes, Refactoring, Tests, Documentation, Configuration, Performance, Security.

### Step 3: Extract Technical Details

For each significant change: purpose, approach, files affected, dependencies, side effects.

### Step 4: Identify Related Issues

Search commits and branch name for #123, GH-456. Link issues, design docs, related PRs.

### Step 5: Assess Testing Strategy

Unit tests, integration tests, manual testing, coverage impact.

### Step 6: Check Breaking Changes

API changes, config changes, dependency version changes, schema changes.

Identify any required migration work such as config changes, rollout order, data backfills, operator actions, or compatibility notes.

### Step 7: Generate The Description

Start from [../../../.github/pull_request_template.md](../../../.github/pull_request_template.md) and follow the wording rules from [code-review-standards](../code-review-standards/). Remove sections that do not apply, and ensure the final PR description covers the same information required by the template:

- **Summary**: 2-3 sentences
- **What Changed**: Group related changes instead of listing files
- **Why**: User or system impact
- **How to Test**: Actual verification performed
- **Breaking Changes**: Only when applicable
- **Migration**: Required rollout, upgrade, backfill, or operator steps
- **Related**: Issues, design docs, or follow-up work

### Step 8: Review and Validate

Ensure completeness, technical accuracy, valid links, and that testing matches actual work. Confirm that the final description stays in sync with [../../../.github/pull_request_template.md](../../../.github/pull_request_template.md) and does not repeat generic review or clean-code checklists from the referenced documents.

## Edge Cases

- **No changes**: Report error, check branch/commits
- **Too many changes**: Summarize categories, detail significant only
- **No tests**: Warn incomplete testing section
- **Multiple unrelated changes**: Suggest splitting PRs

## Related Resources

- [Pull request template](../../../.github/pull_request_template.md)
- [code-review-standards](../code-review-standards/)
- [Coding Conventions in AGENTS.md](../../../AGENTS.md)
