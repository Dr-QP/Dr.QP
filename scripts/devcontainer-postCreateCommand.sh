#!/usr/bin/env bash
set -exuo pipefail

sudo chown -R root:root \
    /opt/ros/overlay_ws/.ansible \
    /opt/ros/overlay_ws/.vscode \
    /opt/ros/overlay_ws/.cache \
    /opt/ros/overlay_ws/build \
    /opt/ros/overlay_ws/install \
    /opt/ros/overlay_ws/lcov \
    /opt/ros/overlay_ws/log \
    /opt/ros/overlay_ws/docs/_build \
    /opt/ros/overlay_ws/.micromamba \
    /uv

# ~/.claude.json can't be backed directly by a named volume (Docker volumes are always
# directory-backed, so mounting one at a file path materializes an empty directory
# there instead of the file Claude Code expects). Persist it as a plain file inside the
# already-mounted drqp-claude volume and symlink it into place instead.
claude_json_target="/root/.claude/claude.json"
if [[ -f /root/.claude.json && ! -L /root/.claude.json ]]; then
    mv /root/.claude.json "$claude_json_target"
elif [[ ! -e "$claude_json_target" ]]; then
    echo '{}' >"$claude_json_target"
fi
ln -sf "$claude_json_target" /root/.claude.json
