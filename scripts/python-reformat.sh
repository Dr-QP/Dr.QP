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
$script_dir/sync-notebooks.sh
nbqa 'ruff format' $root_dir/notebooks/*.ipynb
nbqa 'ruff check --ignore E402 --fix' $root_dir/notebooks/*.ipynb
nbqa 'ruff check --select I' --fix $root_dir/notebooks/*.ipynb
$script_dir/sync-notebooks.sh
