#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/__utils.sh"

rosdep install --from-paths "$sources_dir" --ignore-src -r -y

# rosdep install can bring in some system packages like clang-format and clang-tidy
# which will mess up the alternatives system that has clang from the external llvm install.
# This script will fix it.
"$script_dir/fix-alternatives.sh"
