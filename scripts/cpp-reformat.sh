#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/__utils.sh"

ament_clang_format --config .clang-format --reformat "$sources_dir/"

# It runs, but doesn't fix anything and is very slow
# ament_clang_tidy --fix-errors "$root_dir/build/compile_commands.json"
# --config .clang-tidy
