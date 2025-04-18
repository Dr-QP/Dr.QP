#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/ros/__utils.sh"

source $root_dir/.venv/bin/activate

jupytext --sync "$root_dir/notebooks/*.md" "$root_dir/notebooks/*.ipynb" --quiet "$@"
