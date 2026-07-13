# Spec 01: Unified orchestration and scope resolution

- **Status**: proposed
- **Depends on**: PR #440 (`reformat-consolidation-step1`)
- **Size**: M

## Objective

Provide one repository-level fix/check contract over the focused entry points established by
PR #440. Preserve Super-Linter ownership of Ansible, clang-format, and Prettier; preserve explicit
notebook operations; and remove hardcoded source-scope gaps.

## Ownership contract

The orchestrator delegates rather than invoking formatter binaries independently:

| File class                              | Focused owner                                                     |
| --------------------------------------- | ----------------------------------------------------------------- |
| Ordinary Python                         | `python-reformat.sh` / shared Ruff helper                         |
| Notebook MyST sources and paired output | `notebooks-format.sh` and `notebooks-sync.sh`                     |
| C/C++                                   | `super-linter-local.sh` with generated clang-format settings      |
| Ansible                                 | `super-linter-local.sh` with generated ansible-lint settings      |
| Markdown, YAML, JSON, JSONC             | `super-linter-local.sh` with generated Prettier settings          |
| Heterogeneous non-fixing checks         | Super-Linter plus focused native hooks where deliberately adopted |
| ROS package lint                        | `scripts/with-ros-env.sh` and package ament tests                 |

Do not add a native C++ formatter wrapper, a project-local Prettier formatter path, or a native
ansible-lint autofix path. Those would create a second execution surface for work consolidated
under Super-Linter in PR #440.

## Public commands

Add a thin public contract; exact implementation filenames may change, but the modes must remain
stable:

```text
scripts/format.sh --write [--all | --changed [--base REF]]
scripts/format.sh --check [--all | --changed [--base REF]]
scripts/lint.sh --fast [--all | --changed [--base REF] | PATH ...]
scripts/lint.sh --all
```

- `format.sh --write` runs owners in a deterministic order: ordinary Python, notebook formatting,
  then Super-Linter autofix. Prettier therefore sees notebook Markdown after Jupytext/Ruff.
- `format.sh --check` is read-only and detects every delta the write command would produce.
- `lint.sh --fast` runs deterministic native checks that are valid for the selected files and do
  not require a ROS build. It clearly reports when the authoritative Super-Linter check was not
  run.
- `lint.sh --all` runs Super-Linter checks and the ROS/ament gates through
  `scripts/with-ros-env.sh`.
- The orchestrator must reuse `super-linter-env.sh`; it must not duplicate Super-Linter flags.
- Existing focused scripts remain supported. They are useful for rapid iteration and are not
  deprecated forwarding wrappers.

## Scope resolution

- Build source scope from tracked files plus requested non-ignored untracked files, not a list of
  top-level directories.
- Exclude build, install, log, `.venv`, vendor, generated runtime headers, and `.git` through one
  tested inventory aligned with `super-linter-env.sh` and language configuration.
- Changed mode includes branch changes from the merge base, staged changes, unstaged changes, and
  non-ignored untracked files. CI always supplies the base SHA explicitly.
- Deleted files are ignored and renames use their destination path.
- Configuration changes fan out to the complete affected owner. Examples include `ruff.toml`,
  `.clang-format`, `.prettierrc.yml`, `.prettierignore`, `.ansible-lint.yml`, `.shellcheckrc`,
  `.hadolint.yaml`, `zizmor.yaml`, and `super-linter-env.sh`.
- Empty selections succeed without starting unnecessary tools. Path handling is NUL-safe.
- Add `docs/source/conf.py` and
  `docker/ros/deploy/install-overlay-python-requirements.py` to ordinary Python ownership.

Super-Linter's own changed-file detection may remain the implementation for its owned classes,
provided tests prove it receives the same base and exclusions as the shared resolver. Do not
pretend explicit single-file Super-Linter formatting exists if the container only supports a
changed/full repository scope.

## Notebook handling

Notebook formatting remains explicit:

- `python-reformat.sh` formats ordinary `.py` sources only and does not call notebook scripts.
- `notebooks-format.sh` formats selected notebook sources/code cells through Jupytext and Ruff.
- `notebooks-sync.sh` is the explicitly named pair synchronization entry point.
- CI and the repository orchestrator call notebook formatting as a separate ordered step.
- Pre-commit passes changed notebook Markdown filenames directly to `notebooks-format.sh`.
- Ruff/Jupytext configuration changes select all tracked notebook sources.
- Prettier may format the Markdown layer in the later Super-Linter phase; it does not own Python
  inside notebook cells.

The current notebook helper discovers both `.md` and `.ipynb` when called without arguments.
Clarify and test whether ignored generated `.ipynb` files are inputs, outputs, or both; ensure the
read-only check does not mutate pairs.

## Ruff simplification

Add Ruff import sorting (`I`) to the normal selected rule set and use one format pass plus one
lint/fix pass. Reconcile the uv-locked Ruff behavior with staged Ruff before removing the explicit
third pass.

## Test plan

- Use `./.tmp/quality-entrypoints/` for temporary fixtures.
- Verify every owned file class reaches only its established execution owner.
- Verify Super-Linter flags come only from `super-linter-env.sh`.
- Verify the two omitted Python files enter Ruff scope and vendor/generated files do not.
- Verify changed mode includes committed, staged, unstaged, and untracked changes exactly once.
- Verify configuration changes fan out to every affected tracked file.
- Verify empty scopes and filenames containing spaces.
- Verify `--check` leaves tracked and generated files unchanged on success and failure.
- Verify ordered notebook formatting followed by Prettier is idempotent.
- Verify `python-reformat.sh` never invokes either notebook script.
- Verify `notebooks-format.sh <changed.md>` does not touch unrelated notebook pairs.

## Acceptance criteria

- [ ] The PR #440 ownership matrix is encoded once and documented.
- [ ] One fix command invokes Python, notebooks, and Super-Linter in deterministic order.
- [ ] One read-only check command detects every formatter delta without modifying files.
- [ ] Focused PR #440 scripts remain supported and behaviorally consistent with orchestration.
- [ ] C++, Ansible, and Prettier are not given competing native autofix entry points.
- [ ] Changed/full scope and configuration fan-out are tested.
- [ ] Ordinary Python discovery is complete across tracked first-party files.
- [ ] Notebook formatting remains explicit and ordered separately from Python formatting.
- [ ] Ruff import sorting no longer needs a third pass.
