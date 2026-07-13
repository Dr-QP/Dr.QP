#!/usr/bin/env bash
set -xeuo pipefail

script_dir=$(dirname "$0")
source "$script_dir/__utils.sh"

cd "$root_dir"
echo "$pwd"
ls -al ".git"
ls -al ".git/"
echo $(command -v git || echo "git not found")
echo $(command -v pre-commit || echo "pre-commit not found")

# Install the repository's hooks for this checkout. Re-running this command is
# safe and ensures both staged-file and pre-push checks are available.
pre-commit install --install-hooks --hook-type pre-commit --hook-type pre-push
