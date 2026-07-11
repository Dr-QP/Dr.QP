#!/usr/bin/env bash
#
# Verify Python style compliance with ament_flake8 (the CI gate).
#
# ruff (see python-reformat.sh) is the fast formatter/autofixer, but CI gates
# on ament_flake8, which uses stricter defaults than ruff's config (e.g. its
# own line-length and import-grouping rules). Run this after reformatting to
# catch violations ruff does not report or cannot autofix.
#
# Usage:
#   scripts/python-lint-check.sh [PATH ...]
#
# ament_flake8 gates the ROS 2 packages, so with no arguments this checks
# ./packages. Pass explicit paths (files or directories) to scope the check to
# a single package or file for rapid iteration.
#
# Exits non-zero when ament_flake8 reports any violation.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$script_dir/__utils.sh"

if [ "$#" -gt 0 ]; then
  targets=("$@")
else
  targets=(
    "$sources_dir"
  )
fi

# ament_flake8 is a ROS tool, so make sure the environment is sourced.
exec "$script_dir/with-ros-env.sh" ament_flake8 "${targets[@]}"
