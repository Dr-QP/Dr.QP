---
type: bug
description: The docs workflow path filter ignores pyproject.toml, uv.lock, and the setup-python-venv action, so docs-toolchain bumps merge without a docs build.
generated:
  by: claude-code/opus-5.5
  at: 2026-10-12T12:09:00Z
sources:
- id: docs-workflow
  resource: .github/workflows/docs.yml
- id: pyproject
  resource: pyproject.toml
- id: pr-548
  resource: https://github.com/Dr-QP/Dr.QP/pull/548
- id: pr-539
  resource: https://github.com/Dr-QP/Dr.QP/pull/539
- id: pr-545
  resource: https://github.com/Dr-QP/Dr.QP/pull/545
---

# Bug: docs build skips Python dependency changes

## Symptom

Renovate PRs that change only the docs toolchain show every check green without
building the docs. `Build docs / docs` is `skipping` on #548
(pydata-sphinx-theme 0.22.0 → 0.23.0), #539 (jupytext 1.19.5 → 1.19.6), and on
#545 and #547, which change `.github/actions/setup-python-venv/action.yml`.
`Build docs / Docs build finished` still passes, so a theme or notebook
toolchain regression reaches `main` unnoticed and only shows on the next PR that
touches `docs/**`.

## Reproduction

1. Open a PR that edits only `pyproject.toml` and `uv.lock`, for example bumping
   `pydata-sphinx-theme` in the `docs` dependency group.
2. `gh pr checks <n>` lists `Build docs / docs` as `skipping`.

## Root cause

The `docs` filter in the `paths-filter` job of `.github/workflows/docs.yml`
matches only `docs/**`, the workflow itself, and
`.github/actions/log-debug-stats/**`. The `docs` job installs its toolchain
through `./.github/actions/setup-python-venv` from the `docs` and `notebooks`
dependency groups in `pyproject.toml`, locked in `uv.lock`, so none of those
inputs trigger the build.

## Fix

Unfixed. Add `pyproject.toml`, `uv.lock`, and
`.github/actions/setup-python-venv/**` to the `docs` filter. Notebooks import
`drqp_brain` and `drqp_kinematics` from the `notebooks` group, so consider
`packages/runtime/drqp_brain/**` and `packages/runtime/drqp_kinematics/**` too.
Re-run #548 and #539 after the change, since both are manual-merge and otherwise
look verified.

## Key references

Verified anchor points (line numbers as of 2026-10-12):

- `.github/workflows/docs.yml:33` — `docs` path filter (`docs/**`, workflow,
  log-debug-stats only)
- `.github/workflows/docs.yml:61` — `Setup Python venv` step using
  `./.github/actions/setup-python-venv`
- `pyproject.toml:23` — `docs` dependency group
- `pyproject.toml:27` — `pydata-sphinx-theme==0.22.0`
- `pyproject.toml:49` — `jupytext==1.19.5` in the `notebooks` group
