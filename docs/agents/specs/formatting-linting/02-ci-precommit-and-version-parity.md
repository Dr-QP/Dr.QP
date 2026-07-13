# Spec 02: Serial CI, fast hooks, and version boundaries

- **Status**: proposed
- **Depends on**: 01
- **Size**: L

## Objective

Make CI produce one deterministic formatter result from the owners established in PR #440, and
make pre-commit/pre-push coverage deliberate without starting Super-Linter for every commit.

## Version sources

| Tool/surface                                     | Version source                                                 |
| ------------------------------------------------ | -------------------------------------------------------------- |
| Ruff for full local/CI formatting                | `pyproject.toml` plus `uv.lock`                                |
| Ruff for staged pre-commit formatting            | pinned `ruff-pre-commit` revision                              |
| Jupytext and notebook Ruff pipeline              | notebook dependency group plus `uv.lock`                       |
| clang-format, Prettier, ansible-lint in CI/local | one pinned Super-Linter image used by action and local wrapper |
| ShellCheck/actionlint/gitleaks native hooks      | pinned pre-commit revisions                                    |
| Ament/ROS lint tools                             | pinned development container/ROS image                         |

Separate pins are acceptable only when the surface has a distinct performance role and parity is
tested or its diagnostics are documented as advisory. Do not add a project-local Prettier or
native clang-format/ansible-lint autofix merely to force one version source; the authoritative
versions for those owners are in the Super-Linter image.

## Pre-commit and pre-push behavior

Preserve the PR #440 changed-file design:

- Keep native Ruff format, lint-fix, and import handling for staged ordinary Python.
- Keep `notebooks-format.sh` as a dedicated filename-receiving hook for notebook Markdown.
- Keep standard whitespace, line-ending, merge-conflict, YAML syntax, ShellCheck, actionlint,
  gitleaks, and agent-file validation hooks where their results are valid file-by-file.
- Do not invoke `python-reformat.sh` or start Super-Linter from an ordinary pre-commit hook.
- Preserve the pre-commit contract: if a hook modifies a file, the commit stops for review and
  restaging.

The devcontainer currently installs both `pre-commit` and `pre-push` hook types, but no quality
hook is assigned to the pre-push stage. Add a real pre-push contract or stop claiming that slower
pre-push checks are available.

Recommended pre-push behavior:

- run the authoritative changed-scope Super-Linter check once;
- map changed ROS files to affected packages and run package lint gates once per package;
- allow an explicit full mode for configuration changes or manual verification;
- never run a package/container check once per selected file.

If measured Super-Linter startup cost makes an automatic pre-push hook unacceptable, document one
manual `lint.sh --all` gate and install only hooks that have actual configured stages. This is a
decision to make from timings, not from an assumption that every CI tool belongs in pre-commit.

### Performance evidence

- Record cold/warm timings for ordinary Python, notebook, shell, C++, Ansible, Markdown, and no-op
  commits.
- Record one changed-scope Super-Linter run and affected-package pre-push run.
- Verify unrelated tools do not start for ordinary staged-file hooks.
- Set performance budgets from the measurements and document the selected hook boundary.

## CI formatting behavior

Replace the parallel patch producers with one ordered writer:

```text
ordinary Ruff -> notebook Jupytext/Ruff -> Super-Linter autofix -> combined patch
```

- Notebook formatting must precede Super-Linter so Prettier receives the final Markdown produced by
  the notebook pipeline.
- Run the complete ordered pipeline in one checkout/worktree or pass the output tree serially; do
  not create independent patches from the same base.
- Produce one binary-safe patch and at most one formatter commit.
- Run the read-only checks on the formatted revision/recursive workflow.
- Preserve trusted same-repository PR autofix commits.
- For forks, upload the combined patch and fail with the exact local reproduction command; never
  attempt a remote branch update.
- Preserve skip-tag trust checks and collapse the two formatter commit subjects into one recursive
  skip identity.

Super-Linter retains `FIX_ANSIBLE`, `FIX_CLANG_FORMAT`, and the Prettier fix flags. Serializing CI
must not disable those flags or replace them with native formatter jobs.

## Trigger inventory

The reformat path filter must cover all owned sources and every configuration/entry-point change,
including:

- `ruff.toml`, Python dependency/lock files, `ruff-commands.sh`, and `python-reformat.sh`;
- notebook source/configuration plus `notebooks-format.sh` and `notebooks-sync.sh`;
- `.clang-format`, `.ansible-lint.yml`, `.prettierrc.yml`, `.prettierignore`, `.shellcheckrc`,
  `.hadolint.yaml`, and `zizmor.yaml`;
- `super-linter-env.sh`, `super-linter-local.sh`, `.pre-commit-config.yaml`, and the workflow;
- all tracked first-party paths/extensions owned by those tools.

Remove references to deleted `.github/super-linter-*.env` files. Add a test comparing trigger
declarations with the ownership/config inventory from spec 01.

## Parity tests

- Assert local and GitHub Actions use the same Super-Linter image tag.
- Snapshot or probe the Super-Linter-bundled clang-format, Prettier, and ansible-lint versions so
  upgrades are visible.
- Test representative files through native staged Ruff and uv-backed Ruff; either align output or
  document/reject incompatible drift.
- Test the editor versions only if they are advertised as equivalent to the authoritative gate.
- Ensure the generated autofix and check settings differ only by intentionally fixing/check-only
  flags.

## Test plan

- Seed one defect for every formatter owner and verify the serial writer repairs all defects in one
  idempotent pass.
- Include notebook Markdown requiring both code-cell Ruff changes and outer Prettier changes.
- Verify CI produces one patch/commit and the recursive run produces none.
- Verify a fork receives one artifact and no branch write attempt.
- Run staged representative files and assert unrelated tracked files remain untouched.
- Verify the installed pre-push hook has actual pre-push-stage checks, or verify setup no longer
  installs/advertises that hook type.
- Exercise every trigger configuration and a tracked Python file outside the historical roots.
- Validate pre-commit configuration and test installation in normal checkouts and Git worktrees.

## Acceptance criteria

- [ ] CI runs formatter owners serially in the required notebook-before-Prettier order.
- [ ] CI creates at most one combined formatter patch and commit per source revision.
- [ ] Super-Linter remains the authoritative C++, Ansible, and Prettier autofix/check surface.
- [ ] Pre-commit remains changed-file-scoped and container-free.
- [ ] The installed pre-push behavior matches configured stages and documentation.
- [ ] Separate fast-feedback versions are pinned and parity-tested or marked advisory.
- [ ] Local and CI Super-Linter image versions match.
- [ ] Formatter/configuration changes always trigger the appropriate gate.
- [ ] Fork workflows never attempt to write the source branch.
