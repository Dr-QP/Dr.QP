#!/usr/bin/env bash
set -euo pipefail

workspace="${DEV_WORKSPACE_FOLDER:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"

cd "$workspace"

# UV_PROJECT_ENVIRONMENT keeps the environment on the /uv volume, which is the
# same filesystem as UV_CACHE_DIR. That is what lets uv hardlink packages out of
# its cache instead of copying them; an in-tree .venv would sit on the host bind
# mount and silently fall back to copying.
#
# Unlike the upstream template, this workspace keeps a `.venv` symlink pointing
# at that environment: AGENTS.md tells contributors and agents to run local
# scripts, docs builds, and notebooks through `.venv`, and the VS Code workspace
# resolves its interpreter through it.
if [ -e "$workspace/.venv" ] || [ -L "$workspace/.venv" ]; then
    rm -rf "$workspace/.venv"
fi

# Sync UV with the latest version of the codebase
uv sync --all-groups --all-extras

ln -s "$UV_PROJECT_ENVIRONMENT" "$workspace/.venv"
