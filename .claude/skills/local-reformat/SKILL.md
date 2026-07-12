---
name: local-reformat
description: 'Run every formatter and Super-Linter validation from the reformat GitHub Actions workflow locally. Use when applying repository-wide format fixes, reproducing reformat.yml, running Python formatting, or running Super-Linter Ansible, clang-format, Prettier, and validation passes. Keywords: reformat, formatter, python-reformat, ansible-lint, Ansible, Super-Linter, clang-format, prettier.'
---

# Run the Reformat Workflow Locally

Run the local entry points that mirror the formatter jobs in
[reformat.yml](../../../.github/workflows/reformat.yml): Python formatting,
then Super-Linter's Ansible, clang-format, Prettier, and validation passes.
These commands modify files; inspect the resulting diff and keep only intended
changes.

## When to Use This Skill

- Applying all automatic formatting before committing or opening a pull request
- Reproducing the formatter portions of the `Reformat code` GitHub Actions job
- Formatting C++, Python, Ansible, Markdown, YAML, JSON, JSONC, or GitHub
  Actions files in one workflow
- Investigating a Super-Linter failure locally

## Prerequisites

- Run commands from the repository root.
- Sync the repository virtual environment before the Python formatter if it is
  missing or stale:

  ```bash
  uv sync --frozen --all-groups --all-extras
  ```

- Install and start Docker or Podman before running Super-Linter. The local
  wrapper pulls `ghcr.io/super-linter/super-linter:v8.5.0` unless overridden.
- The shared clang-format configuration is
  [.github/linters/.clang-format](../../../.github/linters/.clang-format).
  Root [.clang-format](../../../.clang-format) is a symlink for ament and
  editor discovery.

## Codex Managed-Sandbox Execution

For this workspace, the Super-Linter container is an approved, required local
validation action. Codex must run `scripts/super-linter-local.sh` with
`sandbox_permissions: "require_escalated"` so Docker daemon access and the
repository mount are available. Do not omit this step or substitute a partial
lint command when opening a pull request.

The Super-Linter local wrapper includes Ansible, clang-format, and Prettier
autofixes and must run with the same elevated sandbox permission.

## Full Workflow

1. Format and autofix Python, notebooks, and scripts. This runs Ruff format,
   Ruff fixes, Ruff import sorting, and notebook synchronization:

   ```bash
   scripts/python-reformat.sh
   ```

2. Run Super-Linter's Ansible, clang-format, Prettier, and other autofix
   passes, then its check pass, using the exact pinned image and configuration
   used by CI:

   ```bash
   scripts/super-linter-local.sh
   ```

   The wrapper runs both passes consecutively. The Actions workflow skips its
   check pass when autofixes produce a patch so it can commit that patch; local
   execution instead checks the newly fixed working tree immediately.

3. Inspect and validate the results:

   ```bash
   git diff --check
   git status --short -- . ':(exclude).tmp'
   git diff -- . ':(exclude).tmp'
   ```

   Super-Linter writes its report to `logs/super-linter-output.md`. Fix any
   remaining findings, then rerun its local wrapper until the check pass exits
   successfully.

## Scope and Options

By default Super-Linter checks the changed files, matching the workflow's
default `validate_all_codebase: false` input. Use `--all` when CI is invoked
with full-repository validation:

```bash
scripts/super-linter-local.sh --all
```

Use these troubleshooting options only when necessary:

```bash
scripts/super-linter-local.sh --log-level DEBUG
scripts/super-linter-local.sh --image ghcr.io/super-linter/super-linter:v8.5.0
```

Do not substitute a newer image merely to make a local result pass: keep the
pinned CI image unless the workflow itself is intentionally being updated.

## Related Entry Points

| Entry point                                                     | CI-equivalent responsibility                                                                |
| --------------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| [python-reformat.sh](../../../scripts/python-reformat.sh)       | Run Ruff format/autofix/import sorting and synchronize notebooks.                           |
| [super-linter-local.sh](../../../scripts/super-linter-local.sh) | Run Super-Linter's Ansible, clang-format, Prettier, and other autofix/check passes.         |
| [super-linter-env.sh](../../../scripts/super-linter-env.sh)     | Generate the shared Ansible, clang-format, Prettier, and validation settings for each pass. |

For Python package linting beyond the reformat workflow, use the
[python-format-lint](../python-format-lint/SKILL.md) skill to verify
`ament_flake8`.
