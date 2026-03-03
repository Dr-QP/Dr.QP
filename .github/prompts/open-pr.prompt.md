---
description: 'Open or update a pull request with a title and body synced to the actual branch changes, following code-review-standards'
name: 'open-pr'
agent: 'agent'
model: 'claude-sonnet-4-5'
tools: ['read', 'execute']
argument-hint: 'base-branch draft(true/false) issue-number'
---

# Open Pull Request

Open a pull request (or update an existing one) with a title and body that are precisely synced to the actual changes on the current branch. Follows [code-review-standards](../skills/code-review-standards/SKILL.md).

## When to Use This Prompt

- Opening a pull request for committed branch changes
- Keeping a PR title and body up to date after new commits
- Replacing boilerplate PR descriptions with accurate, change-driven content
- Ensuring PR documentation reflects what is actually in the branch

## Prerequisites

- Changes are committed to the current branch
- `gh` CLI is authenticated (`gh auth status`)
- Branch has been pushed to the remote (`git push`)

## Inputs

- **Base branch** `${input:base:main}`: Branch to merge into (default: `main`)
- **Draft** `${input:draft:false}`: Open as draft PR (`true`/`false`)
- **Related issue** `${input:issue:}`: GitHub issue number to close (e.g. `42`), optional

## Workflow

### Step 1: Gather branch context

```bash
# Identify current branch and remote base
git rev-parse --abbrev-ref HEAD
git remote get-url origin

# Check if a PR already exists for this branch
gh pr view --json number,title,body,url 2>/dev/null || true
```

**Stop here** if `gh auth status` returns an error — print `gh auth login` and exit.

### Step 2: Analyse the changes

```bash
# Commit log
git log --oneline origin/${input:base:main}..HEAD

# Files changed with status
git diff --name-status origin/${input:base:main}..HEAD

# Full diff (for understanding intent)
git diff origin/${input:base:main}..HEAD

# Stats
git diff --stat origin/${input:base:main}..HEAD
```

**Stop here** if `git log --oneline origin/${input:base:main}..HEAD` returns no output — there is nothing to PR.

Categorise every changed file into one or more buckets:
`Features` | `Bug Fixes` | `Refactoring` | `Tests` | `Documentation` | `Configuration` | `Performance` | `Security`

### Step 3: Generate the PR title

Derive a single concise title from the dominant change category:

- **Imperative mood, lowercase type prefix**: `feat:`, `fix:`, `refactor:`, `docs:`, `test:`, `chore:`, `perf:`
- **Max 72 characters**
- **No trailing period**
- Reflects the most significant change, not a list of files

Examples:
```
feat: add timeout handling to serial communication driver
fix: resolve servo calibration offset at boot
refactor: extract state-machine transitions into dedicated module
docs: document ROS 2 launch parameters for drqp_control
```

### Step 4: Generate the PR body

Write the body using only the sections that apply. Do **not** include empty sections, tables, hidden HTML comments, or AI attribution.

```markdown
<one-line summary identical to the title but as a sentence>

## What Changed

- <imperative bullet: what was added / removed / modified>
- <repeat for each logical change group>

## Why

<Two-to-four sentences explaining the motivation. Focus on the problem
solved or the improvement made, not on implementation mechanics.>

## How to Test

1. <concrete step>
2. <concrete step>
3. Expected result: <description>

## Breaking Changes

<Omit this section entirely if there are none.>

- <what changed and what callers must update>

## Related Issues

<Omit this section entirely if there are none.>

Closes #<number>
Related to #<number>
```

Rules enforced from [code-review-standards](../skills/code-review-standards/SKILL.md):
- Imperative mood throughout ("Add", not "Added" or "Adds")
- No per-file change tables
- No marketing or Copilot attribution
- No invisible Unicode or HTML comments
- All claims must be verifiable against the actual diff

### Step 5: Open or update the PR

**If no PR exists for this branch:**

```bash
gh pr create \
  --base "${input:base:main}" \
  --title "<generated title>" \
  --body "<generated body>"
# Append --draft if ${input:draft:false} is "true"
```

Include `Closes #${input:issue:}` in the body's **Related Issues** section when an issue number is provided (as the Step 4 template already shows). Do **not** pass `--body-file`.

**If a PR already exists:**

```bash
gh pr edit \
  --title "<generated title>" \
  --body "<generated body>"
```

**Stop here** if the `gh` command exits non-zero — report the error message and do not proceed.

### Step 6: Confirm

Print the success report described in [Output Expectations](#output-expectations) below.

## Output Expectations

### Success

```
✅ PR opened/updated: <URL>

Title : <title>
Base  : <base branch>
Draft : <yes/no>

📋 Sections included: <list of non-empty sections>
🔗 Related issues   : <list or "none">
⚠️  Breaking changes : <yes/no>
```

### Failure triggers

- **`gh` not authenticated** — print `gh auth login` and stop before Step 1.
- **No commits ahead of base** — report "No commits ahead of `${input:base:main}`; nothing to PR." and stop after Step 2.
- **PR is already merged or closed** — report its status and stop; do not re-open.
- **`gh` command fails** — report the exact error output and stop; do not retry silently.

## Edge Cases

- **Untracked/unstaged changes exist**: Warn the user; the PR will not include uncommitted work.
- **Many unrelated changes**: Note it in the body summary and suggest splitting into multiple PRs.
- **No tests changed**: Add a note in "How to Test" that no automated test changes were made and describe manual verification steps instead.

## Quality Assurance

Before submitting the `gh` command, verify:

- [ ] Title uses imperative mood and correct type prefix
- [ ] Title is ≤ 72 characters
- [ ] Body contains only sections that have real content
- [ ] Every bullet in "What Changed" corresponds to an actual diff entry
- [ ] "Why" explains motivation, not mechanics
- [ ] "How to Test" has numbered, reproducible steps
- [ ] No tables, no HTML comments, no AI attribution
- [ ] Related issue numbers (if any) exist in the repository

## References

- [code-review-standards skill](../skills/code-review-standards/SKILL.md)
- [generate-pr-description prompt](./generate-pr-description.prompt.md) — use this to draft PR content only; use `open-pr` to also open or update the PR via `gh`
- [Engineering guidelines](../instructions/engineering.instructions.md)
- [GitHub CLI — `gh pr create`](https://cli.github.com/manual/gh_pr_create)
- [GitHub CLI — `gh pr edit`](https://cli.github.com/manual/gh_pr_edit)
