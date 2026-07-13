#!/usr/bin/env bash
set -xeuo pipefail

script_dir=$(dirname "$0")
source "$script_dir/__utils.sh"

cd "$root_dir"

command -v git || echo "git not found"
command -v pre-commit || echo "pre-commit not found"

git config --global --add safe.directory "$root_dir"
git status

# Install the repository's hooks for this checkout. Re-running this command is
# safe and ensures both staged-file and pre-push checks are available.
pre-commit install --install-hooks --hook-type pre-commit --hook-type pre-push || echo "pre-commit install failed. log: $(cat "$HOME/.cache/pre-commit/pre-commit.log")"
