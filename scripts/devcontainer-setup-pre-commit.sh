#!/usr/bin/env bash
set -euo pipefail

script_dir=$(dirname $0)
source "$script_dir/__utils.sh"

cd "$root_dir"

# Install the repository's hooks for this checkout. Re-running this command is
# safe and ensures both staged-file and pre-push checks are available.
pre-commit install --install-hooks --hook-type pre-commit --hook-type pre-push
