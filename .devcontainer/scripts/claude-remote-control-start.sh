#!/usr/bin/env bash
set -euo pipefail

for command in claude tmux; do
  if ! command -v "$command" >/dev/null 2>&1; then
    echo "$command is not installed; skipping Claude Remote Control startup." >&2
    exit 0
  fi
done

# A setup token would mask the saved login and cannot run Remote Control.
if ! auth_status="$(env -u CLAUDE_CODE_OAUTH_TOKEN claude auth status --json 2>/dev/null)"; then
  echo "Claude is not logged in; skipping Remote Control."
  exit 0
fi
if ! auth_method="$(python3 -c '
import json, sys
status = json.load(sys.stdin)
print(status.get("authMethod") if status.get("loggedIn") is True else "none")
' <<<"$auth_status" 2>/dev/null)"; then
  echo "Could not parse 'claude auth status --json'; skipping Remote Control." >&2
  exit 0
fi
if [[ "$auth_method" != "claude.ai" ]]; then
  echo "No claude.ai login (auth method: $auth_method); skipping Remote Control."
  exit 0
fi

session="claude-remote"
workspace="${DEV_WORKSPACE_FOLDER:-$PWD}"

if tmux -L "$session" has-session -t "=$session" 2>/dev/null; then
  echo "Claude Remote Control tmux session '$session' is already running."
  exit 0
fi

# Setup tokens cannot establish Remote Control; keep one out of that process.
tmux_start=(tmux -L "$session" start-server \; new-session -d -s "$session" -c "$workspace" \
  "exec env -u CLAUDE_CODE_OAUTH_TOKEN claude /remote-control")

if ! tmux_error="$("${tmux_start[@]}" 2>&1)"; then
  if tmux -L "$session" has-session -t "=$session" 2>/dev/null; then
    echo "Claude Remote Control tmux session '$session' is already running."
    exit 0
  fi
  printf 'Failed to start Claude Remote Control: %s\n' "$tmux_error" >&2
  exit 1
fi

echo "Started Claude Remote Control in tmux session '$session'."
