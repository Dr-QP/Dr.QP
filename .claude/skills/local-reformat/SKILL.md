---
name: local-reformat
description: 'Run every formatter and Super-Linter validation from the reformat GitHub Actions workflow locally. Use when applying repository-wide format fixes, reproducing reformat.yml, running C++ and Python formatters, or running Super-Linter autofix and checks. Keywords: reformat, formatter, cpp-reformat, python-reformat, Super-Linter, prettier, ament_clang_format.'
---

# Run the Reformat Workflow Locally

Run the local entry points that mirror the formatter jobs in
[reformat.yml](../../../.github/workflows/reformat.yml): C++ formatting,
Python formatting, then Super-Linter's autofix and check passes. These commands
modify files; inspect the resulting diff and keep only intended changes.

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

- Run the C++ formatter through `scripts/with-ros-env.sh`. If ROS 2 is missing
  on the host, follow the workspace escalation procedure instead of retrying
  locally.
- Install and start Docker or Podman before running Super-Linter. The local
  wrapper pulls `ghcr.io/super-linter/super-linter:v8.5.0` unless overridden.

## Full Workflow

1. Format C++ with the same `ament_clang_format` configuration used by CI:

   ```bash
   scripts/with-ros-env.sh scripts/cpp-reformat.sh
   ```

2. Format and autofix Python, notebooks, scripts, and Ansible. This runs Ruff
   format, Ruff fixes, Ruff import sorting, notebook synchronization, and
   `ansible-lint --fix`:

   ```bash
   scripts/python-reformat.sh
   ```

3. Run Super-Linter's autofix pass and its check pass using the exact pinned
   image and configuration files used by CI:

   ```bash
   scripts/super-linter-local.sh
   ```

   The wrapper runs both passes consecutively. The Actions workflow skips its
   check pass when autofixes produce a patch so it can commit that patch; local
   execution instead checks the newly fixed working tree immediately.

4. Inspect and validate the results:

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

| Entry point | CI-equivalent responsibility |
| --- | --- |
| [cpp-reformat.sh](../../../scripts/cpp-reformat.sh) | Run `ament_clang_format --reformat` with [.clang-format](../../../.clang-format). |
| [python-reformat.sh](../../../scripts/python-reformat.sh) | Run Ruff format/autofix/import sorting, synchronize notebooks, and fix Ansible. |
| [super-linter-local.sh](../../../scripts/super-linter-local.sh) | Run Super-Linter's autofix and check configurations in a local container. |
| [super-linter-autofix.env](../../../.github/super-linter-autofix.env) | Enable Prettier and Zizmor fixes. |
| [super-linter-checks.env](../../../.github/super-linter-checks.env) | Enable the non-fixing Super-Linter validation set. |

For Python package linting beyond the reformat workflow, use the
[python-format-lint](../python-format-lint/SKILL.md) skill to verify
`ament_flake8`.
