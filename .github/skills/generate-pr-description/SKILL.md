---
name: generate-pr-description
description: Generate comprehensive pull request description following code-review-standards with change analysis, testing strategy, and migration notes. Use when creating a PR, writing PR description, preparing for code review, or documenting technical decisions.
---

# Generate Pull Request Description

Generate a comprehensive PR description by analyzing the change set and filling the structure defined by [code-review-standards](../code-review-standards/). Use [engineering guidelines](../../instructions/engineering.instructions.md) only for shared quality expectations rather than repeating them here.

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

- **Branch Name** or **Commit Range**: Default `origin/main..HEAD` or `main..<branch>`
- **Related Issues**: GitHub issue numbers
- **Breaking Changes**: Yes/No
- **Migration Steps**: If breaking

## Workflow

### Step 1: Analyze Git Changes

```bash
git diff --name-status <range>
git diff <range>
git log --oneline <range>
git diff --stat <range>
```

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

### Step 7: Generate The Description

Use the template and wording rules from [code-review-standards](../code-review-standards/). Include only sections that are relevant to the change:

- **Summary**: 2-3 sentences
- **What Changed**: Group related changes instead of listing files
- **Why**: User or system impact
- **How to Test**: Actual verification performed
- **Breaking Changes**: Only when applicable
- **Related**: Issues, design docs, or follow-up work

### Step 8: Review and Validate

Ensure completeness, technical accuracy, valid links, and that testing matches actual work. Do not repeat generic review or clean-code checklists from the referenced documents.

## Edge Cases

- **No changes**: Report error, check branch/commits
- **Too many changes**: Summarize categories, detail significant only
- **No tests**: Warn incomplete testing section
- **Multiple unrelated changes**: Suggest splitting PRs

## Related Resources

- [code-review-standards](../code-review-standards/)
- [Engineering guidelines](../../instructions/engineering.instructions.md)
