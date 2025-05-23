#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/__utils.sh"

source $root_dir/.venv/bin/activate

jupytext --sync "$root_dir/docs/source/notebooks/*.md" "$root_dir/docs/source/notebooks/*.ipynb" --quiet "$@"
