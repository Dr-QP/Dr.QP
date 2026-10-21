#!/usr/bin/env bash
set -euo pipefail

# Agent-state primitives shared by the devcontainer hooks and standalone startup.
# AGENTDEV_RUNTIME_MODE=standalone makes every step fatal; the hooks keep CBM nonfatal.
# Usage: setup-agent-state.sh create|links|agent-policy|catalog <dir> [claude-scope]

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=runtime-paths.sh
source "$script_dir/runtime-paths.sh"

# ~/.claude.json lives in the ~/.claude volume and is linked BEFORE the CBM
# install, so the install folds its MCP entry back in. Spec: catalog-lifecycle.
link_claude_json() {
  local target="$agentdev_claude_dir/claude.json"
  mkdir -p "$agentdev_claude_dir"
  if [[ -f $agentdev_claude_json && ! -L $agentdev_claude_json ]]; then
    if agentdev_standalone; then
      # No image-baked copy exists in a standalone home: a real file is the user's.
      mv -f "$agentdev_claude_json" "$target"
    else
      rm -f "$agentdev_claude_json"
    fi
  fi
  if [[ ! -e $target ]]; then
    echo '{}' >"$target"
  fi
  ln -sfn "$target" "$agentdev_claude_json"
}

install_cbm() {
  if agentdev_standalone; then
    "$script_dir/runtime-cbm.sh" install
  else
    # Nonfatal: a failure here would also skip postStart (firewall, keyring, Xpra).
    "$script_dir/codebase-memory-mcp-install.sh" ||
      echo "WARNING: codebase-memory-mcp install failed (exit $?); continuing without it." >&2
  fi
}

ensure_auth_dirs() {
  local agent
  for agent in claude codex; do
    mkdir -p "$agentdev_auth_dir/$agent"
    chmod 700 "$agentdev_auth_dir/$agent"
  done
}

# The image republishes root-home user tooling from system paths; register it for
# whichever account runs here, since a fresh home volume holds none of it.
register_user_tools() {
  local source="${AGENTDEV_GH_STACK_DIR:-/usr/local/lib/agentdev/gh-stack}"
  local extension="${XDG_DATA_HOME:-$HOME/.local/share}/gh/extensions/gh-stack"
  [[ -x $source/gh-stack && -f $source/manifest.yml ]] || return 0
  mkdir -p "$extension"
  ln -sfn "$source/gh-stack" "$extension/gh-stack"
  sed "s#^path: .*#path: $extension/gh-stack#" "$source/manifest.yml" >"$extension/manifest.yml"
}

apply_agent_policy() {
  "$script_dir/configure-codex.py"
  "$script_dir/configure-opencode.py"
}

link_auth() {
  "$script_dir/link-codex-auth.sh"
}

cmd_create() {
  if agentdev_standalone; then
    register_user_tools
  fi
  link_claude_json
  install_cbm
  ensure_auth_dirs
  "$script_dir/preapprove-claude-workspace.sh"
  link_auth
}

cmd_catalog() {
  local catalog="$1" claude_scope="${2:-}"
  "$script_dir/reinstall-agentdev-codex.sh" "$catalog"
  "$script_dir/reinstall-agentdev-claude.sh" "$catalog" ${claude_scope:+"$claude_scope"}
  "$script_dir/reinstall-agentdev-opencode.sh" "$catalog"
}

case "${1:-}" in
  create) cmd_create ;;
  links) link_auth ;;
  agent-policy) apply_agent_policy ;;
  catalog)
    [[ -n ${2:-} ]] || { echo "usage: $(basename "$0") catalog <dir> [claude-scope]" >&2 && exit 2; }
    cmd_catalog "$2" "${3:-}"
    ;;
  *)
    echo "usage: $(basename "$0") create|links|agent-policy|catalog <dir> [claude-scope]" >&2
    exit 2
    ;;
esac
