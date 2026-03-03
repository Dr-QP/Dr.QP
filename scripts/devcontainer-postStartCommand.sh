#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"$script_dir/devcontainer-start-docker.sh"
"$script_dir/../docker/ros/desktop/start-xpra.sh" --background
