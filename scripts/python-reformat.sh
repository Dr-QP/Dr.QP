#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/ros/__utils.sh"

PATH="$PATH:/home/anton/.vscode/extensions/charliermarsh.ruff-2025.14.0-linux-x64/bundled/libs/bin/"
all_python_files=$(find $sources_dir -name "*.py")
ruff format $all_python_files
ruff check --fix $all_python_files
