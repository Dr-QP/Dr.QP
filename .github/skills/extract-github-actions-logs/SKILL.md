---
name: extract-github-actions-logs
description: Extract logs from a GitHub Actions run or job with the GitHub CLI. Use when asked to fetch failing CI logs, inspect a GitHub Actions run, or pull logs from a run or job URL. Keywords: github actions logs, failing ci logs, workflow run logs, job logs, gh run view.
---

# Extract GitHub Actions Logs

Use this skill to pull GitHub Actions logs with `gh`.

## When to Use This Skill

- Fetch logs for a failing GitHub Actions run or job
- Inspect CI from a GitHub Actions URL
- Pull the log for a specific run ID or job ID

## Prerequisites

- `gh` must be installed
- `gh` authentication is required

Always verify authentication first:

```bash
gh auth status
```

If that command fails or shows no authenticated account, stop immediately and tell the user to authenticate first with:

```bash
gh auth login
```

Do not continue until `gh auth status` succeeds.

## Workflow

1. Verify `gh` authentication with `gh auth status`.
2. Identify the repository, run ID, and optional job ID from the user input.
3. Fetch logs with `gh`.

Use these commands:

```bash
gh run view <run-id> --repo <owner>/<repo>
gh run view <run-id> --repo <owner>/<repo> --log
gh run view <run-id> --repo <owner>/<repo> --job <job-id> --log
```

If the user wants only the failure lines, filter the job log:

```bash
gh run view <run-id> --repo <owner>/<repo> --job <job-id> --log | grep -nE "FAILED|FAILURES|AssertionError|ERROR:|Segmentation fault|test_"
```

## Example

Given this failing CI job URL:

`https://github.com/Dr-QP/Dr.QP/actions/runs/26806349222/job/79025184711?pr=366`

Extract:

- repo: `Dr-QP/Dr.QP`
- run ID: `26806349222`
- job ID: `79025184711`

Then run:

```bash
gh auth status
gh run view 26806349222 --repo Dr-QP/Dr.QP --job 79025184711 --log
```

Or, to focus on the likely failure lines:

```bash
gh run view 26806349222 --repo Dr-QP/Dr.QP --job 79025184711 --log | grep -nE "FAILED|FAILURES|AssertionError|ERROR:|Segmentation fault|test_" | tail -n 200
```
