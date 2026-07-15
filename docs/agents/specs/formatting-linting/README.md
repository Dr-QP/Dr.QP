# Formatting and linting consolidation

## Re-evaluated direction

This spike was re-evaluated against the implementation in
[PR #440](https://github.com/Dr-QP/Dr.QP/pull/440), `reformat-consolidation-step1`. That branch is
now the baseline for the remaining work, not an intermediate state to undo.

PR #440 establishes these ownership decisions:

1. Ruff owns ordinary Python formatting and lint fixes through `python-reformat.sh` and the native
   pre-commit hooks.
2. Notebook formatting and notebook-pair synchronization are explicit operations owned by
   `notebooks-format.sh` and `notebooks-sync.sh`. `python-reformat.sh` deliberately does not invoke
   either operation.
3. Super-Linter is the execution owner for Ansible (`ansible-lint`), C/C++ (`clang-format`), and
   Markdown/YAML/JSON/JSONC (`Prettier`) autofixes and checks. It also aggregates ShellCheck,
   Hadolint, actionlint, zizmor, and gitleaks validation.
4. `super-linter-env.sh` generates the settings used by both GitHub Actions and the local wrapper;
   checked-in autofix/check env files are no longer configuration authorities.
5. Pre-commit is a changed-file feedback layer. It runs native Ruff, notebook formatting,
   Prettier, clang-format, ansible-lint, Hadolint, zizmor, ShellCheck, whitespace/merge checks,
   actionlint, gitleaks, and agent-file validation without starting the Super-Linter container.
6. Ament lint remains an additive ROS package gate. Ruff and Super-Linter do not replace package
   lint registration or `ament_flake8` compatibility.

The remaining program should consolidate orchestration and close enforcement gaps while preserving
those owners. In particular, it must not recreate `cpp-reformat.sh`, move Prettier or Ansible out
of Super-Linter, or fold notebook formatting back into `python-reformat.sh`.

## Remaining specs

| Order | Spec                                                                              | Result                                                                |
| ----- | --------------------------------------------------------------------------------- | --------------------------------------------------------------------- |
| 1     | [01 — Unified orchestration and scope](01-ownership-and-entry-points.md)          | One orchestration contract over the PR #440 owners and complete scope |
| 2     | [02 — Serial CI, fast hooks, and versions](02-ci-precommit-and-version-parity.md) | Serial CI output and deliberate fast-hook coverage                    |
| 3     | [03 — ROS package lint gates](03-ros-package-lint-gates.md)                       | Ament coverage is explicit, reproducible, and non-accidental          |
| 4     | [04 — VS Code and agent workflow](04-vscode-and-agent-workflow.md)                | Editor feedback and guidance match repository ownership               |

## Current execution topology after PR #440

| Concern                                   | Local/script                                                          | Pre-commit                                               | CI / package gate                                                    |
| ----------------------------------------- | --------------------------------------------------------------------- | -------------------------------------------------------- | -------------------------------------------------------------------- |
| Ordinary Python                           | `python-reformat.sh`: Ruff format, fix, and explicit import-sort pass | Three native `ruff-pre-commit` hooks                     | `python-reformat.sh`; package `ament_flake8` where registered        |
| Notebook MyST/paired notebooks            | Explicit `notebooks-format.sh` and `notebooks-sync.sh`                | `notebooks-format.sh` receives changed notebook Markdown | Explicit `notebooks-format.sh` after ordinary Python formatting      |
| C/C++                                     | `super-linter-local.sh`: Super-Linter clang-format autofix/check      | Pinned staged clang-format                               | Super-Linter clang-format; additive ament checks where registered    |
| Ansible                                   | `super-linter-local.sh`: Super-Linter ansible-lint autofix/check      | Pinned staged ansible-lint                               | Super-Linter ansible-lint with repository-root offline configuration |
| Markdown, YAML, JSON, JSONC               | `super-linter-local.sh`: Super-Linter Prettier autofix/check          | Pinned staged Prettier plus basic whitespace/YAML checks | Super-Linter Prettier                                                |
| Bash, Dockerfile, workflows, secrets      | `super-linter-local.sh`                                               | ShellCheck, Hadolint, actionlint, zizmor, and gitleaks   | Super-Linter ShellCheck, Hadolint, actionlint, zizmor, and gitleaks  |
| CMake, XML, copyright, ROS-specific style | No unified repository command                                         | None                                                     | `drqp_lint_common`/ament tests, package by package                   |

## What PR #440 resolves

- **Resolved: competing C++ entry points.** `cpp-reformat.sh` is removed; Super-Linter owns the
  clang-format autofix/check path.
- **Resolved: mixed Python/Ansible formatting.** `python-reformat.sh` is Ruff-only and no longer
  invokes ansible-lint.
- **Resolved: implicit notebook work.** Notebook formatting and synchronization have named scripts
  and CI invokes notebook formatting explicitly.
- **Resolved: duplicated Super-Linter configuration.** One generator emits autofix and check
  settings for CI and local execution.
- **Resolved: missing basic commit hooks.** Standard whitespace, merge-conflict, line-ending, and
  ShellCheck hooks are enabled, and devcontainer startup installs hook environments.
- **Resolved: local Super-Linter result discovery.** The wrapper writes the current summary and
  detailed outputs under `log/`.
- **Resolved: generated/vendor scope noise for Super-Linter.** Root configuration and exclusions
  cover build products, virtual environments, vendored sources, and generated runtime headers.

## Remaining issue register

### P0 — required gates can still disagree or be absent

1. **ROS lint registration still has holes.** `drqp_gazebo` declares lint-auto dependencies without
   invoking `ament_lint_auto_find_test_dependencies()`, and `drqp_robot_mcp` declares Python lint
   dependencies without registering corresponding tests.
2. **CI still produces two formatter patches.** Ruff/notebook formatting and Super-Linter run in
   parallel, then their patches are applied and committed separately. Notebook Markdown can be
   touched by the Jupytext/Ruff pipeline and by Prettier, so ordering is still significant.
3. **There is no single check/fix contract.** Users must know when to run Python, notebooks,
   Super-Linter, and ROS package gates separately. Passing one surface does not demonstrate that
   the others pass.

### P1 — scope, hook, and version drift

4. **Python scope remains hardcoded and incomplete.** `docs/source/conf.py` and
   `docker/ros/deploy/install-overlay-python-requirements.py` are outside the current
   `python-reformat.sh` roots, while staged Ruff hooks can receive them.
5. **Ruff still has two pins and three passes.** The uv lock and `ruff-pre-commit` revision differ,
   and import sorting remains a separate invocation because `I` is not selected in `ruff.toml`.
6. **Fast-hook parity needs active maintenance.** Pre-commit now covers the formatter and validator
   classes owned by Super-Linter, but every pin must be reviewed when the Super-Linter image changes.
7. **The installed pre-push hook has no pre-push stages.** Devcontainer setup installs both hook
   types, but the current pre-commit configuration defines no `stages: [pre-push]` quality gate.
8. **Editor tool versions are not authoritative.** clangd, the Prettier extension, and local
   ansible-lint diagnostics may differ from the versions bundled in the pinned Super-Linter image.
9. **Reformat path filters lag the new files.** The workflow still names removed Super-Linter env
   files and does not comprehensively inventory the generator, root linter configs, notebook
   scripts, pre-commit config, and all owned source locations.

### P2 — feedback and documentation gaps

10. **VS Code exposes tools without an ownership distinction.** Ruff and clangd are useful
    save-time feedback, while Super-Linter remains authoritative for C++/Prettier/Ansible; the
    workspace and guidance do not state that difference.
11. **Super-Linter remains broad and container-backed.** This is now an intentional ownership
    choice, but failure routing and inexpensive reproduction need concise documentation.
12. **Human onboarding remains incomplete.** The repository lacks one short sequence for ordinary
    Python changes, notebook changes, heterogeneous changes, and full ROS verification.

## Target ownership matrix

| Files                               | Formatting algorithm | Authoritative execution surface       | Additional checks                                                   |
| ----------------------------------- | -------------------- | ------------------------------------- | ------------------------------------------------------------------- |
| Ordinary Python                     | Ruff                 | uv-backed repository script and CI    | Native staged Ruff; ament_flake8/pep257 for ROS packages            |
| Notebook code cells and pairs       | Jupytext + Ruff      | Explicit notebook scripts and CI step | Pair synchronization check                                          |
| C/C++                               | clang-format         | Super-Linter local wrapper and action | cpplint, cppcheck, and build warnings through ament                 |
| Ansible YAML/Jinja                  | ansible-lint         | Super-Linter local wrapper and action | Syntax/integration tests where applicable                           |
| Markdown, generic YAML, JSON, JSONC | Prettier             | Super-Linter local wrapper and action | actionlint/zizmor for workflows                                     |
| Bash                                | none                 | No autofix owner                      | ShellCheck in pre-commit and Super-Linter                           |
| Dockerfiles                         | none                 | No autofix owner                      | Hadolint in Super-Linter                                            |
| CMake/XML/package manifests         | none                 | No general autofix owner              | ament_lint_cmake/xmllint                                            |
| GitHub Actions/security             | Prettier where valid | Super-Linter for formatting           | actionlint, zizmor, gitleaks, actual workflow execution, and CodeQL |

Multiple additive checkers are expected. A file class must have only one authoritative autofix
surface; editor extensions and pre-commit normalization hooks are feedback layers, not competing
repository formatter owners.

## Recommended migration path

### Phase 1: expose the established owners through one contract

- Add a thin repository orchestrator for fix, check, changed, and full modes. It delegates Python,
  notebook, and Super-Linter work to the PR #440 scripts rather than reimplementing their tools.
- Centralize tracked-file discovery and exclusions, including configuration fan-out.
- Add read-only check modes where the current scripts only autofix.
- Keep `python-reformat.sh`, `notebooks-format.sh`, `notebooks-sync.sh`, and
  `super-linter-local.sh` as supported focused entry points.

### Phase 2: serialize CI and define fast-hook boundaries

- Run notebook formatting after ordinary Python and before Super-Linter so Prettier sees the final
  notebook Markdown.
- Produce at most one patch and one formatting commit from the ordered pipeline.
- Keep pre-commit file-scoped and container-free. Add only measured native checks there; place
  authoritative heterogeneous/full checks in a real pre-push stage or a documented manual command.
- Make path triggers derive from or test against the ownership/config inventory.

### Phase 3: close package gate gaps

- Register every declared first-party ament lint dependency and audit declaration/registration
  parity.
- Adopt and test one vendor policy.
- Bring the full ament lint baseline clean before making the unified full gate required.

### Phase 4: align editors and guidance

- Treat Ruff as the authoritative Python editor path.
- Treat clangd, Prettier, and local Ansible diagnostics as advisory unless their output/version is
  proven equivalent to the Super-Linter-owned gate.
- Add workspace tasks and concise human/agent instructions that invoke repository scripts.

## Decisions intentionally deferred

- **markdownlint adoption**: add a pinned/configured CI gate or document the extension as advisory.
- **generic yamllint adoption**: evaluate only after excluding Ansible and avoiding a second style
  authority beside Prettier.
- **fast-hook performance budget**: measure representative commits and set budgets for the staged
  C++, Ansible, Prettier, Hadolint, and zizmor hooks.
- **clang-tidy**: keep semantic/static analysis outside this formatting program.

## Success measures

- One documented command fixes all formatter-owned tracked files in the required order.
- One documented read-only command checks formatting and fast lint.
- A full command reproduces all required Super-Linter and ROS package gates.
- CI creates at most one formatting commit per source revision.
- Adding a tracked file under a new top-level directory does not silently escape ownership.
- Every declared ROS lint dependency corresponds to a registered test.
