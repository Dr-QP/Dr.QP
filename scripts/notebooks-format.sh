#!/usr/bin/env bash

set -euo pipefail

script_dir=$(dirname "$0")
source "$script_dir/ruff-commands.sh"

# __utils.sh derives and exports root_dir for this script.
# shellcheck disable=SC1091
source "$script_dir/__utils.sh"

script_dir=$(dirname "$0")

files_to_format=("$@")
if [ "${#files_to_format[@]}" -eq 0 ]; then
    mapfile -t files_to_format < <(
        find "$root_dir/docs/source/notebooks" -type f \( -name "*.ipynb" -o -name "*.md" \)
    )
fi

"$root_dir/.venv/bin/jupytext" --sync "${files_to_format[@]}" --quiet --pipe "$ruff_format {}" --pipe "$ruff_lint_fix --ignore E402,F811 {}" --pipe "$ruff_isort {}" --pipe-fmt "py:percent"
