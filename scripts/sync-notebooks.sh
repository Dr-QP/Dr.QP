#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/ros/__utils.sh"

$root_dir/.venv/bin/jupytext --sync notebooks/*.md notebooks/*.ipynb
