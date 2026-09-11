#!/usr/bin/env bash
# Marks this checkout as a safe directory for git. Needed whenever the working
# tree is owned by a different uid than the user running git — the devcontainer
# bind mount and the CI responder job's checkout are both such cases.

set -xeuo pipefail

root_dir="${DEV_WORKSPACE_FOLDER:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"

command -v git || echo "git not found"

git config --global --add safe.directory "$root_dir"
