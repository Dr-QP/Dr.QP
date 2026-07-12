#!/usr/bin/env bash

set -euo pipefail

script_dir=$(dirname "$0")
source "$script_dir/ruff-commands.sh"

script_dir=$(dirname "$0")
"$script_dir"/notebooks-sync.sh --pipe "$ruff_format {}" --pipe "$ruff_lint_fix --ignore E402,F811 {}" --pipe "$ruff_isort {}" --pipe-fmt "py:percent"

