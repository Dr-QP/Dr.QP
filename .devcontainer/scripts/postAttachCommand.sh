#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_scripts="$(cd "$script_dir/../../scripts" && pwd)"

git config --global gpg.ssh.program ssh-keygen

"$script_dir/codebase-memory-mcp-index.sh"

"$script_dir/uv-sync.sh"

# Resolve ROS package dependencies for the workspace. Keeps the developer
# `.venv` bootstrap above separate from the ROS runtime environment.
"$workspace_scripts/ros-dep.sh"

# Refresh the workspace catalog on every editor attachment so newly added agents
# and skills reach every client after a window reload.
# A workspace that declares no marketplace of its own — this one — is a no-op.
"$script_dir/reinstall-agentdev-codex.sh"
"$script_dir/reinstall-agentdev-claude.sh"
"$script_dir/reinstall-agentdev-opencode.sh"
