---
type: codebase
description: Workspace shell/Python helpers — the ROS env wrapper every command goes through, dependency install, lint/format, test retry, and Pi maintenance utilities.
source: scripts
source_digest: sha256:026c581e0a3e981dc29decee3963fe349835d53d2b48b12782060c4e969ef99f
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: scripts
---

# scripts

The top-level helper scripts. A `COLCON_IGNORE` file keeps colcon out of the
directory. `__utils.sh` exports `root_dir` and `sources_dir=$root_dir/packages`
for the others.

## Public surface

- **`with-ros-env.sh <cmd…>`:** sources `setup.bash`, then `exec`s the command.
  This is the required wrapper for `colcon` and `ros2` calls.
- **`setup.bash`:** sources `/opt/ros/$ROS_DISTRO/setup.bash`, then
  `install/local_setup.bash` when it exists. When ROS is missing, it prints
  escalation guidance and fails.
- **`ros-dep.sh`:** runs `apt-get update`, rosdep install over `packages/`, and
  `install-overlay-python-requirements.py`.
- **Formatting and lint:**
  - `python-reformat.sh`: ruff format, lint-fix, and isort over the packages,
    notebooks, scripts, and `py_packages`.
  - `python-lint-check.sh`: `ament_flake8`, the CI gate.
  - `ruff-commands.sh`, `shellcheck-fix.sh`, `super-linter-*.sh`.
- **`colcon-test-retry.sh`:** `colcon test`, then up to
  `COLCON_TEST_MAX_ATTEMPTS` (default 3) retries with
  `--packages-select-test-failures` and pytest `--lf`.
- **Package scaffolds:** `pkg-create-cmake.sh`, `pkg-create-py.sh`.
- **Notebook tooling:** `notebooks-format.sh`, `notebooks-sync.sh`.
- **Pi utilities:**
  - `backup-pi.sh`, `backup-ubuntu.sh`
  - `pistatus.sh`
  - `bno.py`: reads the BNO055 over I2C with the Adafruit driver.
  - `bt_pair.expect`
  - `ser2net/`
- **`vendor-update-all.py`:** updates each vendor package with `git subtree`.

## Invariants & gotchas

- `setup.bash` turns off `set -u` while it sources the ROS setup scripts and
  restores it afterwards. The ROS scripts fail under `nounset`.
- `with-ros-env.sh` clears `$@` before sourcing, so the command's arguments do
  not leak into `setup.bash`.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `scripts/with-ros-env.sh:8` — `set --` before sourcing
- `scripts/setup.bash` — ROS and overlay sourcing, missing-ROS guidance
- `scripts/ros-dep.sh` — dependency install
- `scripts/python-lint-check.sh` — `ament_flake8` gate
- `scripts/colcon-test-retry.sh` — retrying test runner
- `scripts/__utils.sh:3` — `sources_dir`
