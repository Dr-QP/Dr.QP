#!/usr/bin/env bash

set -euo pipefail

script_dir=$(dirname "$0")
source "$script_dir/__utils.sh"

# shellcheck source=/dev/null
source "$root_dir/.venv/bin/activate"

ruff_format='ruff format --quiet'
ruff_lint_fix='ruff check --quiet --fix'
ruff_isort="$ruff_lint_fix --select I" # isort aka organize imports
