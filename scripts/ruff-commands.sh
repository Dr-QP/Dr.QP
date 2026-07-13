#!/usr/bin/env bash

set -euo pipefail

script_dir=$(dirname "$0")
# __utils.sh derives and exports root_dir for this script.
# shellcheck disable=SC1091
source "$script_dir/__utils.sh"

# shellcheck source=/dev/null
# root_dir is exported by __utils.sh above.
# shellcheck disable=SC2154
source "$root_dir/.venv/bin/activate"

# These commands are consumed by the scripts that source this file.
# shellcheck disable=SC2034
ruff_format='ruff format --quiet'
# shellcheck disable=SC2034
ruff_lint_fix='ruff check --quiet --fix'
# shellcheck disable=SC2034
ruff_isort="$ruff_lint_fix --select I" # isort aka organize imports
