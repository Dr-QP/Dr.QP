---
name: python-format-lint
description: 'Format and lint Python in this ROS 2 workspace with ruff first, then verify compliance against ament_flake8 (the CI gate). Use when formatting Python, fixing style/lint violations, organizing imports, or checking why flake8 fails in CI. Keywords: ruff, ament_flake8, flake8, format, lint, isort, style, E501, I201, noqa, python-reformat.'
---

# Python Format & Lint (ruff → ament_flake8)

This workspace uses two Python style tools with distinct roles. Use them in
order: **ruff formats and autofixes, `ament_flake8` is the gate CI enforces.**

- **ruff** — fast formatter + autofixer, configured by [ruff.toml](../../../ruff.toml)
  (line-length 99, single quotes, isort with `force-sort-within-sections`,
  `known-third-party` list). Fixes most issues mechanically.
- **`ament_flake8`** — the linter the per-package `test_flake8.py` runs and the
  one **CI actually gates on**. It uses its own defaults that differ from ruff
  and from stock `flake8`, so it can flag things ruff leaves alone.

Never judge compliance with pure `flake8` — its stock defaults (79-char limit,
stock isort grouping) produce false positives that do not match this repo and
do not fail CI.

## When to Use This Skill

- Formatting or cleaning up Python after editing source or tests
- A `test_flake8` / `ament_flake8` check fails in CI and you need to reproduce
  and fix it locally
- Organizing imports, fixing line length, or resolving `I201` import-group errors
- Deciding whether a `# noqa` is justified

## Workflow

1. **Format and autofix with ruff.** Run the bundled reformat script from the
   repo root — it runs `ruff format`, `ruff check --fix`, and isort across
   packages, scripts, notebooks, and Ansible:

   ```bash
   scripts/python-reformat.sh
   ```

   For a tighter loop on specific files, invoke ruff directly (config is picked
   up automatically):

   ```bash
   .venv/bin/ruff format path/to/file.py
   .venv/bin/ruff check --fix path/to/file.py
   ```

2. **Verify against `ament_flake8`.** Run the bundled check script. `ament_flake8`
   gates the ROS 2 packages, so with no arguments it checks `./packages`; pass
   explicit paths to scope it to one package or file for speed:

   ```bash
   scripts/python-lint-check.sh                                   # all packages
   scripts/python-lint-check.sh packages/runtime/drqp_brain       # one package
   scripts/python-lint-check.sh path/to/file.py                   # one file
   ```

   Equivalently, run the package's own gate test:

   ```bash
   scripts/with-ros-env.sh python3 -m pytest \
     packages/runtime/<pkg>/test/test_flake8.py
   ```

3. **Fix remaining violations by hand.** Anything `ament_flake8` reports that
   ruff did not autofix must be fixed in the source. Use a targeted
   `# noqa: <rule>` **only** for a formatter-required incompatibility (e.g.
   `E203` on a slice); never disable a rule for a whole file or package. For
   `I201` import-group mismatches, add the package to `lint.isort.known-third-party`
   in [ruff.toml](../../../ruff.toml) so ruff's grouping matches `ament_flake8`.

4. **Confirm clean.** Re-run step 2 until it reports `No problems found`.

## Scripts

| Script | Purpose |
| ------ | ------- |
| [python-reformat.sh](../../../scripts/python-reformat.sh) | ruff format + `ruff check --fix` + isort across packages, scripts, notebooks, Ansible |
| [python-lint-check.sh](../../../scripts/python-lint-check.sh) | Run `ament_flake8` (the CI gate) over given paths, or all of `./packages` by default |

## Troubleshooting

| Symptom | Cause | Fix |
| ------- | ----- | --- |
| `flake8` locally flags `E501` at 79 chars but CI is green | You ran stock `flake8`, not `ament_flake8` | Use `scripts/python-lint-check.sh`; the repo limit is 99 |
| `I201 Missing newline between import groups` | ruff's isort grouping disagrees with `ament_flake8` | Add the package to `known-third-party` in [ruff.toml](../../../ruff.toml), then re-run `python-reformat.sh` |
| `ament_flake8: command not found` | ROS environment not sourced | The check script wraps it in `with-ros-env.sh`; run the script rather than the bare command |
| ruff reports nothing but CI `test_flake8` fails | ruff and `ament_flake8` rule sets differ | Always verify with step 2 before assuming clean |

## References

- Python coding conventions: [AGENTS.md](../../../AGENTS.md) (Python section)
- ruff configuration: [ruff.toml](../../../ruff.toml)
