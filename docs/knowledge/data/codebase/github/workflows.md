---
type: codebase
description: GitHub Actions CI/CD — primary-checks orchestrator, image builds, colcon build/test in two containers, deploy image, docs, CodeQL, reformat, knowledge-base and agent-file validation, and the AI responder.
source:
- .github/workflows
- .github/actions
source_digest: sha256:206d1c38589a6fa9eba3936cc1f5d3892567bb1499383985027e0839a6444ce5
verified:
  by: claude-code/opus-5.5
  at: 2026-09-27T00:00:00Z
stale_after: 2026-11-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-27T00:00:00Z
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
5. `merge-all`.

Artifacts: `colcon-logs-<arch>`, `colcon-test-reports-<arch>`, and the
`devcontainer-*` equivalents.

## Depends on

- [docker/ros](../docker/ros.md) (both images)
- [scripts](../scripts.md) (`ros-dep.sh`, `setup.bash`)

## Invariants & gotchas

- CI colcon output is reduced. A failing job log names the package, not the
  failure; the details are in the uploaded colcon log and report artifacts.
- The arm64 and amd64 roles swap through the `AMD_ONLY` and `ARM_ONLY`
  repository variables.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `.github/workflows/primary-checks.yml:46` — `ci` job calling `ci.yml`
- `.github/workflows/ci.yml:143` — `ros-ci`
- `.github/workflows/ci.yml:176` — colcon build command
- `.github/workflows/ci.yml:193` — colcon test command
- `.github/workflows/ci.yml:223` — `dev-container-ci`
- `.github/workflows/ci.yml:334` — `build-deploy-image`
