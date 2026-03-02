---
name: generate-pr-description
description: Generate comprehensive pull request description following code-review-standards with change analysis, testing strategy, and migration notes. Use when creating a PR, writing PR description, preparing for code review, or documenting technical decisions.
---

# Generate Pull Request Description

Generate a comprehensive, high-quality PR description that follows code-review-standards and engineering best practices.

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

### Step 7: Generate PR Description

Use [code-review-standards](../code-review-standards/SKILL.md) template. Include:

- **Summary**: 2-3 sentences
- **Changes**: By category (Features, Bug Fixes, Refactoring, Tests, Documentation, Configuration)
- **Technical Details**: Approach, key files, dependencies, assumptions
- **Testing Strategy**: Unit, integration, manual, coverage
- **Breaking Changes**: YES/NO, migration steps, deployment notes
- **Risk Assessment**: Risks and mitigation
- **Related**: Closes #, related to #, design doc link
- **Checklist**: Code follows conventions, tests pass, docs updated, etc.

### Step 8: Review and Validate

Ensure completeness, technical accuracy, valid links, testing matches actual work.

## Edge Cases

- **No changes**: Report error, check branch/commits
- **Too many changes**: Summarize categories, detail significant only
- **No tests**: Warn incomplete testing section
- **Multiple unrelated changes**: Suggest splitting PRs

## Related Resources

- [code-review-standards](../code-review-standards/SKILL.md)
- [Engineering guidelines](../../instructions/engineering.instructions.md)
