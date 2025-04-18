#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/ros/__utils.sh"

source $root_dir/.venv/bin/activate

ruff_format='ruff format --quiet'
ruff_lint_fix='ruff check --quiet --fix'
ruff_isort="$ruff_lint_fix --select I" # isort aka organize imports

$ruff_format $sources_dir $root_dir/notebooks
$ruff_lint_fix $sources_dir $root_dir/notebooks
$ruff_isort $sources_dir  $root_dir/notebooks

script_dir=$(dirname $0)
$script_dir/sync-notebooks.sh --pipe "$ruff_format {}" --pipe "$ruff_lint_fix --ignore E402,F811 {}" --pipe "$ruff_isort {}" --pipe-fmt "py:percent"
