#!/usr/bin/env bash
set -euo pipefail

if [ -d "/opt/ros/overlay_ws/.venv" ]; then
    rm -rf /opt/ros/overlay_ws/.venv
fi

# Sync UV with the latest version of the codebase
uv sync --all-groups --all-extras

ln -s "$UV_PROJECT_ENVIRONMENT" /opt/ros/overlay_ws/.venv
