#!/usr/bin/env bash

set -xeuo pipefail

script_dir=$(dirname "$0")
source "$script_dir/ruff-commands.sh"

script_dir=$(dirname "$0")

files_to_format="$@"
if [ -z "$files_to_format" ]; then
    files_to_format="$(find "$root_dir/docs/source/notebooks" -type f \( -name "*.ipynb" -o -name "*.md" \))"
fi

jupytext --sync $files_to_format --quiet --pipe "$ruff_format {}" --pipe "$ruff_lint_fix --ignore E402,F811 {}" --pipe "$ruff_isort {}" --pipe-fmt "py:percent"
