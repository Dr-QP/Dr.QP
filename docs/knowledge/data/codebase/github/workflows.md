---
type: codebase
description: GitHub Actions CI/CD — primary-checks orchestrator, image builds, colcon build/test in two containers, deploy image, docs, CodeQL, reformat, knowledge-base and agent-file validation, and the AI responder.
source:
- .github/workflows
- .github/actions
source_digest: sha256:719f4da22ebb178a2912e713bee255ae240383eea40a0ecc6cb35dcb591c810d
verified:
  by: claude-code/opus-5.5
  at: 2026-10-21T01:35:27Z
stale_after: 2027-01-19
generated:
  by: claude-code/opus-5.5
  at: 2026-10-21T01:35:27Z
sources:
- id: code
  resource: .github/workflows
---

# github/workflows

The CI/CD definitions and their composite actions (`.github/actions/`: Docker
build, push, and merge; paths-filter; test-result publishing; Python venv setup;
the Claude responder).

## Public surface

- `primary-checks.yml` runs on push, pull request, merge queue, and dispatch. It
  calls these reusable workflows:
  - `reformat.yml`
  - `ci.yml`
  - `docs.yml`
  - `codeql.yml`
- Standalone workflows:
  - `validate-knowledge-base.yml` and `validate-agent-files.yml`, on PR, push,
    and merge queue.
  - `ai-responder.yml`, on issue and review events.
  - `act-docker-*`, `delete-old-containers`, and
    `validate-super-linter-tool-versions`.

## How it works

`ci.yml` runs these jobs in order:

1. `paths-filter`.
2. `build-dev-image`, per architecture, then `merge-dev-image`.
3. Two test jobs on the dev image:
   - `ros-ci`: `ros-dep.sh`, then
     `colcon build --mixin coverage-pytest ninja rel-with-deb-info --symlink-install`,
     then `colcon test` with `PYTEST_ADDOPTS="-rA"`, then LLVM coverage export
     and a Codecov upload.
   - `dev-container-ci`: the same in the devcontainer. The matrix pairs one
     architecture with `sim_test_mode: slow`, which sets `DRQP_TEST_MODE`.
4. `build-deploy-image` and `merge-deploy-image`.
5. `promote-images`, after both test jobs.
6. `merge-all`.

The merge jobs push only the `sha-<commit>` candidate tag
(`docker/metadata-action` with `candidate: 'true'`). `docker/promote` later
points the event's moving tags at the tested digest and skips any tag that
already names a descendant commit. `act-docker-build-image.yml` promotes each
`act-<distro>-` image the same way after its act tests.

`paths-filter` diffs a push to the base branch against the last commit the
calling workflow passed on, falling back to the previous push when that lookup
fails; every caller job needs `actions: read` for it.

Artifacts: `colcon-logs-<arch>`, `colcon-test-reports-<arch>`, and the
`devcontainer-*` equivalents.

## Depends on

- [docker/ros](../docker/ros.md) (both images)
- [scripts](../scripts.md) (`ros-dep.sh`, `setup.bash`)

## Invariants & gotchas

- CI colcon output is reduced. A failing job log names the package, not the
  failure; the details are in the uploaded colcon log and report artifacts.
- `primary-checks.yml` cancels an in-progress run except on `main`, where runs
  queue so `edge` keeps advancing.
- The arm64 and amd64 roles swap through the `AMD_ONLY` and `ARM_ONLY`
  repository variables.
- The Claude responder gets the `agentdev` skills only by running the three
  devcontainer lifecycle hooks in its job container
  (`.github/actions/run-claude-responder/action.yml:106`). The image stages the
  catalog without installing it, and `.claude/settings.json` does not enable it.
  Without them the reviewer cannot load `agentdev:pr-review`.
- `claude-code-action` merges its `settings` input into
  `~/.claude/settings.json` one level deep, so a project `enabledPlugins` key
  would replace the hook-enabled plugins. The responder deep-merges the user
  settings into that input first (`action.yml:137`).

## Key references

Verified anchor points (line numbers as of 2026-10-21):

- `.github/workflows/primary-checks.yml:18` — `cancel-in-progress` except `main`
- `.github/workflows/primary-checks.yml:56` — `ci` job calling `ci.yml`
- `.github/workflows/ci.yml:145` — `ros-ci`
- `.github/workflows/ci.yml:178` — colcon build command
- `.github/workflows/ci.yml:195` — colcon test command
- `.github/workflows/ci.yml:225` — `dev-container-ci`
- `.github/workflows/ci.yml:336` — `build-deploy-image`
- `.github/workflows/ci.yml:407` — `promote-images`
- `.github/workflows/act-docker-build-image.yml:202` — act image `promote`
- `.github/actions/docker/metadata-action/action.yml:59` — candidate `sha` tag
- `.github/actions/docker/promote/action.yml:66` — descendant-tag guard
- `.github/actions/paths-filter/action.yml:63` — `last-pass` base lookup
