#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/__utils.sh"

ament_clang_format --config .clang-format --reformat "$sources_dir/"
