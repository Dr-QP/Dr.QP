#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/ros/__utils.sh"

ruff format "$sources_dir/"
ruff check --fix "$sources_dir/"
