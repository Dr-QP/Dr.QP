---
type: bug
description: Renovate's post-upgrade pre-commit run fails on the agent-desktop bump and hides which hook failed, most likely because hadolint-docker cannot see Renovate's clone through the host Docker socket.
generated:
  by: claude-code/opus-5.5
  at: 2026-10-12T12:09:00Z
sources:
- id: post-upgrade
  resource: scripts/renovate-post-upgrade.sh
- id: renovate-workflow
  resource: .github/workflows/renovate.yml
- id: pre-commit-config
  resource: .pre-commit-config.yaml
- id: pr-541
  resource: https://github.com/Dr-QP/Dr.QP/pull/541
- id: renovate-run
  resource: https://github.com/Dr-QP/Dr.QP/actions/runs/38239750533
---

# Bug: Renovate post-upgrade fails on Dockerfile changes

## Symptom

PR #541 (agent-desktop digest `b902c3e` → `13715ef`) carries a failing
`renovate/artifacts` status and an "Artifact update problem" comment:
`Command failed: scripts/renovate-post-upgrade.sh`. The captured output ends
after `Installed 230 packages` and never names the failing hook. The status
blocks auto-merge, so the agent-desktop group, and the jazzy-ros-desktop rebuild
it is meant to trigger, stays stuck. The Renovate run log at `LOG_LEVEL: info`
shows only an empty `artifactErrors` entry.

## Reproduction

1. Let Renovate update `renovate/agent-desktop` (run 38239750533, commit
   `de75aa1`).
2. `renovate/artifacts` is `failure` on the head commit.
3. Locally, on the same commit,
   `SKIP=no-commit-to-branch uv run --frozen pre-commit run --files` over the
   three changed files passes every hook, so the failure is specific to the
   Renovate job environment.

## Root cause

Two parts.

1. **Output hidden (confirmed).** `renovate-post-upgrade.sh` runs
   `pre-commit run`, which reports hook results on stdout. Renovate's artifact
   comment shows stderr, so the failing hook's name and message are lost.
2. **Failing hook (likely, unconfirmed).** #541 is the only open Renovate PR
   that changes a Dockerfile, and `hadolint-docker` is the only hook that runs
   through `docker run`. The Renovate job runs inside the agent-desktop
   container with the host's `/var/run/docker.sock` mounted, and Renovate's
   clone lives under `/tmp/renovate/repos/...` inside that container. pre-commit
   bind-mounts the clone's path into the hadolint container, but the host daemon
   resolves that path on the host, where it does not exist, so hadolint cannot
   find the Dockerfile. #544, which changes only workflow files, passed the same
   script in the same run.

## Fix

Unfixed. First redirect the pre-commit output to stderr in
`renovate-post-upgrade.sh` (`... pre-commit run ... >&2`) so the next failure
names its hook, then tick the rebase box on #541 to confirm the failing hook. If
it is `hadolint-docker`, skip it in the post-upgrade run
(`SKIP=...,hadolint-docker`); Super-Linter, which pins Hadolint, still lints
Dockerfiles in CI. Alternatively, switch to a non-docker hadolint hook.

## Key references

Verified anchor points (line numbers as of 2026-10-12):

- `scripts/renovate-post-upgrade.sh:33` — first
  `uv run --frozen pre-commit run`, stdout not redirected
- `scripts/renovate-post-upgrade.sh:35` — retry `pre-commit run`
- `.pre-commit-config.yaml:44` — `hadolint-docker` hook
- `.github/workflows/renovate.yml:53` — job `container:` (agent-desktop image,
  host Docker socket)
- `.github/renovate.json:11` — `docker/**` in post-upgrade `fileFilters`
