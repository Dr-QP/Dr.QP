#!/usr/bin/env bash

set -euo pipefail

script_dir=$(dirname "$0")

# These commands are consumed by the scripts that source this file.
# shellcheck disable=SC2034
ruff_format="$script_dir/super-linter/ruff format --quiet"
# shellcheck disable=SC2034
ruff_lint_fix="$script_dir/super-linter/ruff check --quiet --fix"
# shellcheck disable=SC2034
ruff_isort="$ruff_lint_fix --select I" # isort aka organize imports
