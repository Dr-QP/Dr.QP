#!/usr/bin/env bash
# Starts the OpenCode web UI from the Coder agent start; see spec/opencode-web.
set -euo pipefail

for command in opencode tmux; do
  if ! command -v "$command" >/dev/null 2>&1; then
    echo "$command is not installed; skipping OpenCode web startup." >&2
    exit 0
  fi
done

session="opencode"
workspace="${DEV_WORKSPACE_FOLDER:-$PWD}"
command_helper="${BASH_SOURCE[0]%/*}/opencode-web-command.sh"
link="$("$command_helper" link)"
hostname="$("$command_helper" coder-hostname)"
password_note="$("$command_helper" coder-note)"

if tmux -L "$session" has-session -t "=$session" 2>/dev/null; then
  echo "OpenCode web tmux session '$session' is already running: $link$password_note"
  exit 0
fi

pane_command="$("$command_helper" coder-pane)"
tmux_start=(tmux -L "$session" start-server \; new-session -d -s "$session" -c "$workspace" \
  "$pane_command")

if ! tmux_error="$("${tmux_start[@]}" 2>&1)"; then
  if tmux -L "$session" has-session -t "=$session" 2>/dev/null; then
    echo "OpenCode web tmux session '$session' is already running: $link$password_note"
    exit 0
  fi
  printf 'Failed to start OpenCode web: %s\n' "$tmux_error" >&2
  exit 1
fi

echo "Started OpenCode web in tmux session '$session' on $hostname:4096: $link$password_note"
