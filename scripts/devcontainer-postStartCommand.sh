#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ENABLE_FIREWALL=${ENABLE_FIREWALL:-"false"}

"$script_dir/devcontainer-start-docker.sh"
"$script_dir/devcontainer-firewall.sh"
"$script_dir/../docker/ros/desktop/start-xpra.sh" --background
"$script_dir/workspace-extensions.sh"
# Optional: only configure docker-pass when the plugin is available.
if command -v docker >/dev/null 2>&1 && docker pass --help >/dev/null 2>&1; then
    "$script_dir/devcontainer-setup-pass.sh"
fi
