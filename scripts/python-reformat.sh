#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/__utils.sh"

notebooks_dir="$root_dir/docs/source/notebooks"
tool_scripts_dir="$root_dir/scripts"

source $root_dir/.venv/bin/activate

ruff_format='ruff format --quiet'
ruff_lint_fix='ruff check --quiet --fix'
ruff_isort="$ruff_lint_fix --select I" # isort aka organize imports

all_sources="$sources_dir $notebooks_dir $tool_scripts_dir"
$ruff_format $all_sources
$ruff_lint_fix $all_sources
$ruff_isort $all_sources

script_dir=$(dirname $0)
$script_dir/sync-notebooks.sh --pipe "$ruff_format {}" --pipe "$ruff_lint_fix --ignore E402,F811 {}" --pipe "$ruff_isort {}" --pipe-fmt "py:percent"

ansible-lint --fix $root_dir/docker/ros/ansible
