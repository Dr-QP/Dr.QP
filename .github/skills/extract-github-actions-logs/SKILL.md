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

Use the bundled helper script to parse GitHub Actions URLs:

- [parse-actions-url.sh](./scripts/parse-actions-url.sh)

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
2. If the user provides a GitHub Actions URL, parse it with the helper script.
3. Identify the repository, run ID, and optional job ID from the helper's shell-safe `REPO=`, `RUN_ID=`, and `JOB_ID=` output.
4. Fetch logs with `gh`.

Use these commands:

```bash
gh run view <run-id> --repo <owner>/<repo>
gh run view <run-id> --repo <owner>/<repo> --log
gh run view <run-id> --repo <owner>/<repo> --job <job-id> --log
```

Use the helper script when the input is a GitHub Actions URL:

```bash
./scripts/parse-actions-url.sh --url '<github-actions-url>'
./scripts/parse-actions-url.sh --url '<github-actions-url>' --format command --log
```

If the URL is a run URL, fetch the whole run log.

If the URL is a job URL, fetch the job log.

If the user wants only the failure lines, filter the job log:

```bash
gh run view <run-id> --repo <owner>/<repo> --job <job-id> --log | grep -nE "FAILED|FAILURES|AssertionError|ERROR:|Segmentation fault|test_"
```

Or generate that command from the helper script:

```bash
./scripts/parse-actions-url.sh --url '<github-actions-job-url>' --format command --log --grep-failures
```

If the user needs the uploaded colcon logs or xUnit reports from the run, download the artifacts with `gh run download`.

For this repository's ROS CI job, the artifacts are named:

- `colcon-logs-<arch>`
- `colcon-test-reports-<arch>`

If you need to discover the exact artifact names for a run first:

```bash
gh api repos/<owner>/<repo>/actions/runs/<run-id>/artifacts | grep -o '"name":"[^"]*"' | grep 'colcon-'
```

Use these commands:

```bash
gh run download <run-id> --repo <owner>/<repo> -n colcon-logs-<arch> -D ./artifacts
gh run download <run-id> --repo <owner>/<repo> -n colcon-test-reports-<arch> -D ./artifacts
```

Or download both at once:

```bash
gh run download <run-id> --repo <owner>/<repo> -n colcon-logs-<arch> -n colcon-test-reports-<arch> -D ./artifacts
```

## Example

Given this workflow run URL:

`https://github.com/Dr-QP/Dr.QP/actions/runs/26806349222`

Run:

```bash
gh auth status
./scripts/parse-actions-url.sh --url 'https://github.com/Dr-QP/Dr.QP/actions/runs/26806349222' --format command --log
```

Then run the emitted `gh run view ... --log` command to fetch the whole run log.

To inspect and download the ROS CI colcon artifacts for the same run:

```bash
gh api repos/Dr-QP/Dr.QP/actions/runs/26806349222/artifacts | grep -o '"name":"[^"]*"' | grep 'colcon-'
gh run download 26806349222 --repo Dr-QP/Dr.QP -n colcon-logs-amd64 -D ./.tmp/actions-run-26806349222
```

If `colcon-test-reports-<arch>` is present in the artifact list, download it the same way:

```bash
gh run download 26806349222 --repo Dr-QP/Dr.QP -n colcon-test-reports-amd64 -D ./.tmp/actions-run-26806349222
```

If the failing job ran on a different architecture, replace `amd64` with the matching matrix value such as `arm64`.

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

The same job URL can be parsed with:

```bash
./scripts/parse-actions-url.sh --url 'https://github.com/Dr-QP/Dr.QP/actions/runs/26806349222/job/79025184711?pr=366'
```
