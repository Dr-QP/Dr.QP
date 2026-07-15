#!/usr/bin/env bash

set -euo pipefail

script_dir=$(dirname "$0")
source "$script_dir/ruff-commands.sh"

# __utils.sh derives and exports root_dir for this script.
# shellcheck disable=SC1091
source "$script_dir/__utils.sh"

notebooks_dir="$root_dir/docs/source/notebooks"
tool_scripts_dir="$root_dir/scripts"
py_packages_dir="$root_dir/py_packages"

all_sources=("$sources_dir" "$notebooks_dir" "$tool_scripts_dir" "$py_packages_dir")
$ruff_format "${all_sources[@]}"
$ruff_lint_fix "${all_sources[@]}"
$ruff_isort "${all_sources[@]}"
