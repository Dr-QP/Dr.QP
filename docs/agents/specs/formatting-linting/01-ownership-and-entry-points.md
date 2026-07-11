# Spec 01: Formatting ownership and repository entry points

- **Status**: proposed
- **Depends on**: nothing
- **Size**: M

## Objective

Create one explicit ownership matrix and stable, non-CI-specific commands for formatting and
linting. Preserve each language-native tool and the ability to invoke it directly.

## Design

Add these public commands:

```text
scripts/format.sh --write [--all | --changed [--base REF] | --staged | PATH ...]
scripts/format.sh --check [--all | --changed [--base REF] | --staged | PATH ...]
scripts/lint.sh --fast [--all | --changed [--base REF] | --staged | PATH ...]
scripts/lint.sh --all
```

- `format.sh` dispatches each tracked file to exactly one formatter: Ruff, clang-format,
  ansible-lint, or Prettier.
- `lint.sh --fast` runs deterministic file-level checks that do not require a build: Ruff,
  ansible-lint, clang-format check, Prettier check, and the configured shell/Docker/workflow
  checkers.
- `lint.sh --all` additionally runs the ROS/ament lint gates through `scripts/with-ros-env.sh`.
- `--check` and all lint modes are read-only and fail if a formatter would change a file.
- `--changed` computes the union of branch changes from the merge base, staged changes, unstaged
  changes, and non-ignored untracked files. It accepts `--base REF`; its local default is the
  remote default branch with a documented `main` fallback. CI always passes its base explicitly.
- `--staged` reads the index only. Pre-commit normally passes its filename list directly, avoiding
  a second Git scan, but the mode is useful for manual reproduction.
- Deleted files are ignored. Renames use the destination path. Configuration changes deliberately
  expand the affected owner to its complete tracked scope: for example, `ruff.toml` selects all
  Python/notebook sources and `.clang-format` selects all C/C++ sources.
- Paths are derived from `git ls-files` plus requested untracked files. Generated, build, install,
  log, venv, and vendor exclusions live in one shared implementation.
- Empty file sets succeed without invoking tools. Filenames containing spaces are handled safely.

## Notebook handling

Preserve the special behavior currently implemented by `python-reformat.sh` and
`sync-notebooks.sh`:

- ordinary `.py` files under `docs/source/notebooks` use the normal Ruff path;
- tracked notebook `.md` sources use Jupytext's `--pipe` flow so Ruff formats and fixes Python code
  cells, including the existing `E402`/`F811` notebook exceptions;
- only changed notebook sources are passed in changed/staged/path modes;
- each selected MyST source is synchronized with its generated, ignored `.ipynb` pair;
- `docs/source/notebooks/jupytext.toml`, Ruff configuration, or notebook formatter script changes
  select every tracked notebook source;
- the implementation passes an explicit NUL-safe file list to Jupytext and does not leave the
  current quoted wildcard as the only selection mechanism;
- Prettier may format the Markdown layer after Jupytext/Ruff formats code cells, but it does not
  own Python inside notebook cells.

`scripts/python-reformat.sh --changed` remains available during migration and delegates to the
same scope resolver and notebook-aware helper.

Split the current mixed scripts into focused helpers. The exact directory is implementation
choice, but names must reveal ownership, for example:

```text
scripts/quality/format-python.sh
scripts/quality/format-cpp.sh
scripts/quality/format-ansible.sh
scripts/quality/format-prettier.sh
```

Keep `python-reformat.sh`, `cpp-reformat.sh`, and `super-linter-local.sh` as deprecated forwarding
wrappers for at least one migration cycle. Remove `ansible-lint --fix` from the Python helper.

Update Ruff configuration so import sorting is part of the normal lint/fix selection. Ruff should
need one formatting pass and one lint/fix pass, not a separate isort invocation.

## Test plan

- Shell tests create files in `./.tmp/quality-entrypoints/`; never use `$TMPDIR`.
- Verify dispatch for one file of every owned type and verify Ansible YAML never reaches Prettier.
- Verify a Python file in a new top-level directory is discovered.
- Verify vendor, build, install, log, and venv files are excluded.
- Verify `--check` leaves `git status --porcelain` unchanged on both success and failure.
- Verify paths containing spaces and an empty changed-file set.
- Verify `--changed` includes committed branch changes, index changes, worktree changes, and
  untracked files exactly once.
- Verify a changed notebook source formats only its code cells, synchronizes its ignored `.ipynb`
  pair, and does not touch unrelated notebooks.
- Verify a Ruff or Jupytext configuration change selects all tracked notebook sources.
- Verify both omitted Python files identified by the review enter the Ruff scope.
- Verify the deprecated wrapper output is identical to its new focused helper.

## Acceptance criteria

- [ ] The ownership matrix is encoded once and documented.
- [ ] `format.sh --write --all` is idempotent.
- [ ] `format.sh --check --all` is read-only and detects any formatter delta.
- [ ] `lint.sh --fast --all` is read-only and reports the owning tool for failures.
- [ ] Changed/staged modes avoid scanning or rewriting unrelated owned files.
- [ ] Notebook-aware changed formatting preserves Jupytext synchronization and Ruff cell fixes.
- [ ] Python formatting no longer runs ansible-lint.
- [ ] Ruff import sorting no longer needs a third pass.
- [ ] Existing entry points remain as temporary forwarding wrappers.
