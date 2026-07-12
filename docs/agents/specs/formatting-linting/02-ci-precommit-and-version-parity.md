# Spec 02: CI, pre-commit, and tool-version parity

- **Status**: proposed
- **Depends on**: 01
- **Size**: L

## Objective

Make CI and pre-commit call the shared repository contract with deliberately pinned tool versions.
Keep CI formatter commits, while removing parallel formatter patches and retaining Super-Linter as
the sole owner of Ansible formatting and checks.

## Version sources

| Tool                           | Version source                                                         |
| ------------------------------ | ---------------------------------------------------------------------- |
| Ruff, pre-commit               | `pyproject.toml` plus `uv.lock`                                        |
| Prettier                       | repository `package.json` plus a committed lockfile                    |
| clang-format/ament tools       | pinned development container/ROS image, with an asserted major version |
| Super-Linter-hosted checks, including ansible-lint | one Super-Linter release used by both action and local wrapper |

Do not retain an independent `ruff-pre-commit` version. Local pre-commit hooks should call the
shared scripts and use the uv environment installed by the documented bootstrap. Configure the
Prettier VS Code extension to use the project binary where supported.

## Pre-commit behavior

- Automatically install both hook types after the devcontainer's dependency setup:

  ```text
  pre-commit install --install-hooks --hook-type pre-commit --hook-type pre-push
  ```

  Installation must be idempotent, work in Git worktrees, and must not change Git configuration.

- Use two staged-file dispatcher hooks in order: `format staged files`, then `lint staged files`.
  Pass filenames through; do not use `pass_filenames: false` for ordinary formatter/linter hooks.
- The format dispatcher runs every applicable native formatter: notebook Jupytext/Ruff first,
  ordinary Ruff, clang-format, then Prettier. It never scans unrelated file classes. Super-Linter
  owns Ansible formatting through `ansible-lint --fix`.
- The fast lint dispatcher runs every correctly file-scoped check currently enforced in CI,
  including Ruff, ament_flake8 on selected Python, clang-format check, relevant ament file
  linters, Prettier check, ShellCheck, Hadolint, actionlint, zizmor, gitleaks, and agent file
  validation. Super-Linter owns the Ansible `ansible-lint` check. Tools with no selected inputs do
  not start.
- Prefer native/project binaries for hooks. Do not start the Super-Linter container on every
  commit; Super-Linter remains the CI aggregation/reproduction surface for the same underlying
  heterogeneous checks.
- The pre-push hook maps changed files to affected ROS packages and runs the slower package-level
  ament/colcon lint suite once per affected package. A manual `--all` remains available. Checks
  that cannot produce a correct file-scoped result belong here rather than slowing every commit.
- Preserve the normal pre-commit contract: if formatting changes a file, the commit stops so the
  developer can review and stage it. Document `SKIP=<hook>` and `--no-verify` as exceptional escape
  hatches, not the normal workflow.

### Performance requirements

- A commit touching no owned files starts no formatter/linter subprocesses beyond the dispatcher.
- A typical Python-only commit does not start Ansible, clang, Prettier, Docker, or ROS package
  tests; it still runs both Ruff and the file-scoped ament_flake8 compatibility check.
- Notebook formatting receives only the selected notebook sources and their generated pairs.
- Expensive package checks run once at pre-push for affected packages, not once per file.
- Capture cold and warm timings for representative Python, C++, Ansible, Markdown, notebook, and
  no-op commits. Set the final performance budget from those measurements and record it in the
  contributor guidance.

## CI behavior

Replace the two parallel formatting producers in `reformat.yml` with one serial formatting writer:

```text
scripts/format.sh --write --changed --base <pull-request-base-sha>
```

For pushes to `main` and manual full validation, use `scripts/format.sh --write --all`. Pull
requests normally use the explicit base SHA plus index/worktree inputs so CI rewrites only files in
the change. A formatter configuration change remains in changed mode but the scope resolver fans
out to every file owned by that formatter. Notebook sources still take the notebook-aware path
from spec 01.

The job produces one combined binary-safe patch after all formatter owners finish. The existing
commit job applies that patch and creates one formatting commit on a trusted PR source branch.
Recursive-commit detection uses one commit message. If the PR comes from a fork, upload the same
patch as an artifact and fail with the exact changed-only local command; never attempt a remote
branch update.

Super-Linter remains useful for ansible-lint, ShellCheck, Hadolint, actionlint, zizmor, and
gitleaks. Keep `VALIDATE_ANSIBLE` and `FIX_ANSIBLE` enabled as its sole formatter exception;
disable any other formatter/fix flags once the serial native writer owns those file classes. The
local Super-Linter wrapper remains a supported exact reproduction path for those checks.

After a formatter commit, the existing recursive run executes the normal check-only fast lint and
ROS package gates against the formatted revision. This preserves the current write-in-CI workflow
without merging native and Super-Linter patches.

Update path filters so every formatter config, dependency lock, shared script, editor ownership
setting, and owned file extension triggers the gate. Add a test that compares the declared trigger
list with the ownership/config inventory.

## Test plan

- Seed a formatting defect for each owner in `./.tmp`, verify the shared CI command reports it,
  and verify the local write command repairs it.
- Run pre-commit against staged Python, C++, Ansible, and Markdown samples and assert unrelated
  tracked files remain untouched.
- Run a staged notebook sample and assert only that source and its generated pair change.
- Record cold/warm timings for each representative staged-file class and verify unrelated tools
  are not launched.
- Verify devcontainer bootstrap installs pre-commit and pre-push hooks idempotently in a normal
  checkout and a Git worktree.
- Assert the Ruff version used by pre-commit equals `.venv/bin/ruff --version`.
- Assert the project Prettier version used by the extension/CLI equals the lockfile version.
- Assert local and CI Super-Linter release declarations match.
- Exercise path filtering for `ruff.toml`, `.clang-format`, `.prettierrc.yml`, `.prettierignore`,
  `.editorconfig`, lockfiles, quality scripts, and `Dr.QP.code-workspace`.
- Verify a trusted PR receives exactly one combined formatting commit and its next CI run does not
  create another.
- Verify a fork receives a patch artifact and no attempted branch write.

## Acceptance criteria

- [ ] Ansible formatting and checking run only through Super-Linter; no other formatter is run by
      both Super-Linter and a native formatting job.
- [ ] Pre-commit does not run a whole-tree formatter for an ordinary staged file.
- [ ] Devcontainer setup installs performant staged-file and affected-package hooks.
- [ ] Every current formatter and linter runs at the cheapest hook stage where its result is valid.
- [ ] Ruff and Prettier versions have one repository lock source each.
- [ ] Local and CI Super-Linter use the same release.
- [ ] CI runs one deterministic formatting writer and creates at most one combined formatter
      commit per source revision.
- [ ] Changed-only CI and pre-commit preserve notebook formatting/synchronization behavior.
- [ ] Formatter configuration changes always trigger the gate.
