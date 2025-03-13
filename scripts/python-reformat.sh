#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/ros/__utils.sh"

PATH="$PATH:/home/anton/.vscode/extensions/charliermarsh.ruff-2025.14.0-linux-x64/bundled/libs/bin/"

ruff format $sources_dir
ruff check --fix $sources_dir
ruff check --select I --fix $sources_dir # isort aka organize imports
