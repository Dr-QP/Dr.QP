#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -x /usr/local/bin/init-firewall.sh ]; then
    sudo /usr/local/bin/init-firewall.sh
else
    echo "WARNING: init-firewall.sh not found in image; skipping firewall setup"
fi
"$script_dir/devcontainer-start-docker.sh"
"$script_dir/../docker/ros/desktop/start-xpra.sh" --background
"$script_dir/workspace-extensions.sh"
