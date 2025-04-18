#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/ros/__utils.sh"

source $root_dir/.venv/bin/activate

ruff format $sources_dir
ruff check --fix $sources_dir
ruff check --select I --fix $sources_dir # isort aka organize imports


ruff format $root_dir/notebooks
ruff check --fix $root_dir/notebooks
ruff check --select I --fix $root_dir/notebooks

script_dir=$(dirname $0)
$script_dir/sync-notebooks.sh --pipe "ruff format {}" --pipe "ruff check --ignore E402,F811 --fix {}" --pipe "ruff check --select I --fix {}" --pipe-fmt "py:percent"
