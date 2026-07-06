#!/usr/bin/env bash

set -euo pipefail

script_dir=$(dirname "$0")
source "$script_dir/__utils.sh"

function capture_auth() {
  local source="$1"
  local target="$2"
  if [ -f "$source" ] && [ ! -s "$source" ] ; then
      mv -f "$source" "$target"
      ln -s "$target" "$source"
  fi
}

capture_auth f "$HOME/.codex/auth.json" "$root_dir/.codex/auth.json"
capture_auth f "$HOME/.claude/.credentials.json" "$root_dir/.claude/.credentials.json"
