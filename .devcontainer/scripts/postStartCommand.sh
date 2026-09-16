#!/usr/bin/env bash
# This script is run after the container is started, but before the VSCode extension host is started.
# This allows to avoid race conditions and setup safe environment for the extension host to run in.

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_scripts="$(cd "$script_dir/../../scripts" && pwd)"

"$script_dir/codebase-memory-mcp-start.sh"

"$script_dir/setup-git-safe-directory.sh"
"$script_dir/setup-pre-commit.sh"
"$script_dir/setup-keyring.sh"
"$script_dir/firewall.sh"

# Headless callers — the CI responder job — opt out of the remote desktop
# nothing will connect to. Unset (the devcontainer default) starts it as before.
if [[ -n "${AGENTDEV_SKIP_XPRA:-}" ]]; then
  echo "AGENTDEV_SKIP_XPRA is set; skipping Xpra startup."
else
  /start-xpra.sh --background
fi

"$workspace_scripts/workspace-extensions.sh"
"$script_dir/start-virtualhere.sh"
"$script_dir/configure-codex.py"

# Repairs the shared auth.json symlink if a `codex logout` during this container's
# previous run destroyed it; see link-codex-auth.sh for why that can happen.
"$script_dir/link-codex-auth.sh"
