#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/__utils.sh"

# Check if .vscode/extensions exists
if [ -d "$root_dir/.vscode/extensions" ]; then
    echo "$root_dir/.vscode/extensions already exists. Pulling."
    cd "$root_dir/.vscode/extensions"
    git pull
    exit 0
fi

# Clone vscode-extensions
git clone https://github.com/Dr-QP/vscode-extensions.git ${root_dir}/.vscode/extensions
