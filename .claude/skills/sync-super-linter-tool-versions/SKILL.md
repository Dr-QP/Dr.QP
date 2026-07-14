---
name: sync-super-linter-tool-versions
description: 'Synchronize pre-commit hooks and local lint tools with the pinned Super-Linter image. Use when upgrading Super-Linter, fixing a tool-version sync failure, or updating Prettier, clang-format, Ruff, Gitleaks, Actionlint, Zizmor, Hadolint, Ansible Lint, or ShellCheck. Keywords: Super-Linter versions, pre-commit versions, tool sync, lint tool upgrade, zizmor.'
---

# Sync Super-Linter Tool Versions

Treat the image configured by
[super-linter-local.sh](../../../scripts/super-linter-local.sh) as the source
of truth for versions shared by Super-Linter, pre-commit, and local hooks.
The validator reads the image directly, so do not add a manually maintained
version table.

## Workflow

1. Update the Super-Linter image consistently in
   [super-linter-local.sh](../../../scripts/super-linter-local.sh) and both
   `super-linter/super-linter@...` references in
   [reformat.yml](../../../.github/workflows/reformat.yml).

2. Run the validator to print the image's versions and identify every stale
   pin:

   ```bash
   scripts/validate-super-linter-tool-versions.sh
   ```

3. Update the matching values in
   [.pre-commit-config.yaml](../../../.pre-commit-config.yaml): the explicit
   Prettier `additional_dependencies` version, plus the Clang Format, Ansible
   Lint, Hadolint, Ruff, ShellCheck, Gitleaks, and Actionlint repository
   revisions. Preserve the `shellcheck-py` wrapper's fourth version component;
   its first three components must match Super-Linter's ShellCheck binary.

4. Update `ZIZMOR_VERSION` in
   [zizmor-pre-commit.sh](../../../scripts/zizmor-pre-commit.sh). This wrapper
   makes the local `zizmor` pre-commit hook deterministic.

5. Re-run the validator. It must report every tool as `OK` before considering
   the upgrade complete. Then run the affected hooks or
   [super-linter-local.sh](../../../scripts/super-linter-local.sh) if the
   version upgrade can change formatting or lint findings.

## Boundaries

- Do not infer versions from the latest upstream releases; the pinned
  Super-Linter image is authoritative.
- Do not change unrelated pre-commit hooks such as whitespace checks, which
  are not supplied by Super-Linter.
- Do not change the image tag only in one location: the validator rejects a
  mismatch between the local wrapper and the GitHub Actions workflow.
