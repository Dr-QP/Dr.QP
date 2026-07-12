# Formatting and linting tooling review

- **Status**: proposed
- **Reviewed**: 2026-07-11
- **Scope**: repository configuration, local scripts, pre-commit, VS Code/devcontainer,
  Super-Linter, and ROS package lint tests
- **Outcome**: keep language-native formatters and VS Code workflows, but give every file type
  one formatter owner, one checker contract, and shared repository entry points

## Executive summary

The repository has capable tools, but the execution surfaces have evolved independently. The
main problem is not the number of tools; formatting and linting are different jobs and ROS adds
useful package-level gates. The problem is that ownership, file scope, versions, and invocation
order differ between local scripts, pre-commit, VS Code, Super-Linter, and `colcon test`.

The target should be:

1. Language-native tools own formatting: Ruff, clang-format, ansible-lint, and Prettier.
2. Repository scripts are the public contract used by humans, agents, pre-commit, and CI.
3. Super-Linter becomes a check-only host for the remaining heterogeneous checks; it does not
   compete with native formatter jobs.
4. Ament lint remains the ROS package integration gate. It verifies rules that Ruff and
   clang-format do not cover, but every package must actually register its declared linters.
5. VS Code remains a first-class fast feedback path. Extensions use repository configuration and,
   where supported, repository-pinned binaries.
6. CI keeps writing formatting fixes to trusted PR branches, but all formatter owners run serially
   and produce one deterministic patch and commit.

The implementation is split into four independently shippable specs:

| Order | Spec                                                                             | Result                                                        |
| ----- | -------------------------------------------------------------------------------- | ------------------------------------------------------------- |
| 1     | [01 — Ownership and entry points](01-ownership-and-entry-points.md)              | One formatter/checker matrix and complete file scopes         |
| 2     | [02 — CI, pre-commit, and version parity](02-ci-precommit-and-version-parity.md) | The same commands and versions on every automation surface    |
| 3     | [03 — ROS package lint gates](03-ros-package-lint-gates.md)                      | Ament coverage is explicit, reproducible, and non-accidental  |
| 4     | [04 — VS Code and agent workflow](04-vscode-and-agent-workflow.md)               | Save-time feedback and concise guidance for people and agents |

## Accepted workflow decisions

- CI continues to write and commit formatting fixes for trusted pull-request branches.
- CI runs formatter owners serially and creates one combined patch/commit, not one patch per
  formatting system.
- Devcontainer setup installs pre-commit and pre-push hooks automatically.
- Commit-time hooks operate on staged filenames and run all valid file-scoped formatters/linters.
- Slower package-level ROS lint runs at pre-push for affected packages and remains available as an
  explicit full command.
- Local and CI entry points support changed-only operation. Full-tree operation remains available
  for baseline cleanup, configuration migrations, main, and manual validation.
- Changed notebook sources retain the existing Jupytext synchronization plus Ruff code-cell
  formatting; changed-only mode must not degrade notebooks to ordinary Markdown formatting.

## Current execution topology

| Concern                                   | Local/script                                                                            | Pre-commit                                                                | VS Code                                                                              | CI / package gate                                                    |
| ----------------------------------------- | --------------------------------------------------------------------------------------- | ------------------------------------------------------------------------- | ------------------------------------------------------------------------------------ | -------------------------------------------------------------------- |
| Python format/lint                        | `python-reformat.sh`: Ruff format, Ruff fix, separate Ruff isort; then ansible-lint fix | Three Ruff hooks pinned to 0.11.2, then the whole-repository local script | Ruff extension; formatter selected, format-on-save disabled                          | `python-reformat.sh`; package `ament_flake8` tests where registered  |
| C/C++ format                              | `super-linter-local.sh`: Super-Linter clang-format autofix                              | No C++ formatting hook                                                    | clangd formatter selected, format-on-save not enabled                                | Super-Linter clang-format autofix/check; ament clang-format in packages using `drqp_lint_common` |
| Ansible                                   | Coupled to `python-reformat.sh` as `ansible-lint --fix`                                 | Runs indirectly on effectively every commit through the local Python hook | Red Hat Ansible diagnostics use `.venv/bin/ansible-lint`                             | Runs indirectly in the Python reformat job                           |
| Markdown, generic YAML, JSON/JSONC        | No pinned native local command; `super-linter-local.sh` requires Docker/Podman          | None                                                                      | Prettier and markdownlint extensions recommended, but no language formatter settings | Super-Linter runs Prettier autofix then check                        |
| Bash, Dockerfile, GitHub Actions, secrets | Super-Linter container                                                                  | actionlint and gitleaks only                                              | Syntax-focused extensions; no complete CI-equivalent task                            | Super-Linter: ShellCheck, Hadolint, actionlint, zizmor, gitleaks     |
| CMake, XML, copyright, static C++         | ROS/ament commands are available but have no unified repository check script            | None                                                                      | Ament tasks exist for cpplint and lint_cmake                                         | `drqp_lint_common` through `ament_lint_auto`, package by package     |

## What is working well

- `.clang-format`, `ruff.toml`, `.prettierrc.yml`, `.prettierignore`, and the Super-Linter env
  files are repository-owned configuration rather than hidden CI arguments.
- Ansible files are excluded from Prettier, avoiding a formatter fight with ansible-lint.
- `scripts/super-linter-local.sh` deliberately loads the same two env files as CI and handles Git
  worktrees by mounting the common Git directory.
- `.editorconfig` establishes LF endings, final newlines, whitespace trimming, and basic indent
  sizes independently of formatter availability.
- `drqp_lint_common` gives CMake packages a reusable ROS lint dependency set and injects the root
  `.clang-format` into the packages that include `ClangFormatConfig.cmake`.
- The Python agent guidance correctly distinguishes Ruff's formatter/autofixer role from
  `ament_flake8`'s ROS gate role.

## Evidence from read-only verification

Commands were run from the repository root without modifying tracked files.

| Check                                                         | Result                                                                                            |
| ------------------------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| `.venv/bin/pre-commit validate-config`                        | Passed                                                                                            |
| Ruff 0.15.5 `format --check` over all identified Python roots | One omitted file would be reformatted: `docker/ros/deploy/install-overlay-python-requirements.py` |
| Ruff lint with configured rules                               | Passed                                                                                            |
| Ruff import check (`--select I`)                              | Failed for the omitted deploy script and `docs/source/conf.py`                                    |
| ansible-lint 26.3.0 over `docker/ros/ansible`                 | Passed: 56 processed files, no findings                                                           |
| `ament_clang_format --config .clang-format packages/`         | Passed: no C/C++ divergence                                                                       |
| `scripts/python-lint-check.sh`                                | Failed: 26 findings across 389 Python files                                                       |

The ament failures are particularly useful evidence of the split contract:

- one 106-character launch description triggers `E501`, although Ruff explicitly ignores E501
  on the assumption that the formatter handles line wrapping;
- 25 test helper classes trigger the ament `CNL100` class-newline plugin, which the configured
  Ruff rules neither report nor fix;
- `drqp_gazebo` declares `ament_lint_auto` and `drqp_lint_common` but never calls
  `ament_lint_auto_find_test_dependencies()`;
- `drqp_robot_mcp` declares `ament_flake8`, `ament_pep257`, and `ament_copyright`, but has no
  corresponding lint test modules.

Thus a full-tree manual checker can fail while normal package tests do not necessarily register
the same checks. Declaring a test dependency is not evidence that a test exists.

## Issue register

### P0 — enforcement can disagree or be absent

1. **ROS lint registration has holes.** `drqp_gazebo` does not invoke its declared lint-auto
   dependency, and `drqp_robot_mcp` does not register its declared Python linters.
2. **CI has two formatter authorities.** The native reformat job and Super-Linter run in parallel,
   create independent patches, and later attempt to merge them. A file matched by both systems
   can produce patch-order or conflict behavior rather than a deterministic pipeline.
3. **There is no single clean-tree command.** Passing a local formatter, a staged hook, or an
   editor action does not demonstrate that CI and package linters will pass.

### P1 — scope and version drift

4. **Python scope is hardcoded and incomplete.** `docs/source/conf.py` and
   `docker/ros/deploy/install-overlay-python-requirements.py` are tracked Python files outside
   `python-reformat.sh`'s directory list. The staged Ruff hooks can see them, creating different
   behavior by invocation path.
5. **Ruff executes redundantly and at two versions.** The locked venv currently resolves Ruff
   0.15.5; pre-commit pins 0.11.2 and then the local whole-tree script runs Ruff again. Import
   sorting is a third Ruff pass because `I` is not in the root selected rule set.
6. **Ansible is hidden inside the Python formatter.** This causes unrelated Python or repository
   changes to rewrite Ansible files and makes the script name and agent guidance misleading.
7. **Prettier has no repository-pinned local binary.** CI receives the version bundled in the
   Super-Linter image while VS Code normally receives the extension's bundled version.
8. **C/C++ version ownership is implicit.** clang-format comes from the dev image/ROS environment;
   clangd formatting is configured in VS Code, but the expected tool version is not stated or
   checked.
9. **Changed-file path filtering omits configuration triggers.** Changes to files such as
   `ruff.toml`, `.editorconfig`, or workspace formatter settings are not all explicit reasons to
   run the format job.

### P2 — feedback and documentation gaps

10. **Recommended extensions do not describe the repository contract.** Python and C++ have
    selected default formatters, but save-time behavior is off; Prettier has no language-scoped
    defaults; Ansible has diagnostics but no formatter task; markdownlint is recommended without
    a corresponding CI markdownlint gate.
11. **Super-Linter owns unrelated roles.** It is simultaneously a formatter, lint aggregator,
    security checker, PR summary producer, and local Docker-only workflow. This makes failures
    harder to reproduce with the underlying tool.
12. **Human and agent onboarding is incomplete.** `CONTRIBUTING.md` says to follow project style
    but gives no commands. Only Python has a dedicated agent workflow; C++, Ansible, Prettier,
    and Super-Linter ownership is undocumented.
13. **The CI write path is more complex than necessary.** Writing formatter commits is an
    intentional workflow, but two independent patches, two commit identities, recursive-skip
    handling, and fork behavior make it harder to reason about. One serial patch can preserve
    autofix commits without the patch-order ambiguity.

## Target ownership matrix

| Files                                     | Formatter owner                      | Fast checker                               | Integration/additional checks                                                  |
| ----------------------------------------- | ------------------------------------ | ------------------------------------------ | ------------------------------------------------------------------------------ |
| Python, including notebook paired sources | Ruff                                 | Ruff                                       | ament_flake8 and ament_pep257 for ROS packages; notebook sync check            |
| C/C++                                     | clang-format                         | clang-format check                         | cpplint, cppcheck, and package build warnings through ament                    |
| Ansible YAML/Jinja                        | ansible-lint `--fix`                 | ansible-lint                               | playbook syntax/integration tests where applicable                             |
| Markdown                                  | Prettier                             | Prettier check                             | markdownlint only if a repository config and CI gate are intentionally adopted |
| Generic YAML, JSON, JSONC                 | Prettier                             | Prettier check plus parser-specific checks | actionlint/zizmor for workflows; optional yamllint only after rule alignment   |
| Bash                                      | none initially                       | ShellCheck                                 | script-specific tests                                                          |
| Dockerfiles                               | none                                 | Hadolint                                   | image build                                                                    |
| CMake/XML/package manifests               | no general autoformatter initially   | ament_lint_cmake/xmllint                   | configure/build/rosdep validation                                              |
| GitHub Actions/security                   | no generic formatter beyond Prettier | actionlint, zizmor, gitleaks               | actual workflow execution and CodeQL                                           |

This deliberately permits more than one checker for a language when the checks are additive. It
does not permit more than one formatter to rewrite the same file class.

## Recommended migration path

### Phase 1: make ownership and scopes explicit

- Add `scripts/format.sh` and `scripts/lint.sh` as stable public entry points.
- Split Python, Ansible, C/C++, and Prettier implementations into independently callable helpers.
- Build scope from tracked files and explicit exclusions, not a hand-maintained list of top-level
  directories.
- Support `--changed`, `--staged`, and explicit paths. Changed notebook Markdown must retain the
  current Ruff-through-Jupytext code-cell formatting and paired notebook synchronization.
- Keep old script names as temporary forwarding wrappers so existing muscle memory and CI do not
  break in the same change.
- Add check modes before changing CI behavior.

### Phase 2: make automation call the same contract

- Install pre-commit and pre-push hooks automatically in the devcontainer. The commit hook passes
  staged filenames to all applicable formatters and fast linters; the push hook runs slower ROS
  checks only for affected packages.
- Use the uv-locked Ruff and ansible-lint. Add a project-local, lockfile-pinned Prettier and point
  the VS Code extension at it where supported.
- Run one serial native formatting write pass in CI. On pull requests, format changed files and
  expand to all files owned by a tool when that tool's configuration changes; use an explicit full
  run for main/manual validation.
- Produce and commit one combined formatter patch. Remove Super-Linter formatter flags and retain
  it only for heterogeneous checks.

### Phase 3: close package gate gaps

- Register ament lint in every package that declares it and remove declarations that are not used.
- Decide and document vendor policy once: imported vendor code should normally be excluded from
  repository formatting while retaining its upstream package tests.
- Bring the repository to a clean `scripts/lint.sh --all` baseline, including the current ament
  findings, before making the unified gate required.

### Phase 4: restore fast editor workflows and guidance

- Add language-scoped VS Code formatter settings and an explicit opt-in or checked-in
  format-on-save policy.
- Add VS Code tasks for `Format changed files`, `Check formatting`, `Fast lint`, and `Full ROS
lint`, all backed by repository scripts.
- Add a concise contributor section and one cross-language agent skill. Keep language-specific
  troubleshooting in focused references such as the existing Python skill.

## Decisions intentionally deferred

- **markdownlint adoption**: either add a pinned CLI/config and enforce it in CI or remove it as a
  repository recommendation. Do not leave editor-only policy unexplained.
- **generic yamllint adoption**: evaluate it only after excluding Ansible and aligning with
  Prettier; adding a second YAML style authority without that work recreates the current problem.
- **pre-commit performance budget**: record warm timings during implementation, then set budgets
  for typical staged-file hooks and affected-package pre-push checks based on measured container
  performance.
- **clang-tidy**: it is a semantic/static-analysis project, not a formatting consolidation task.
  Keep it outside this program unless separately scoped with compile database and baseline work.

## Success measures

- One documented command fixes every formatter-owned tracked file.
- One documented command checks formatting and fast lint without modifying files.
- A full command reproduces all required ROS package lint gates.
- Pre-commit, CI, and VS Code use the same config files and compatible pinned tool versions.
- Adding a tracked file under a new top-level directory does not silently escape formatting.
- CI creates at most one formatting commit per source revision and never merges formatter patches.
- Every declared ROS lint dependency corresponds to a registered test, verified by an automated
  package audit.
