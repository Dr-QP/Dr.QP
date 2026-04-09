#!/bin/bash
set -euo pipefail

# Only run in Claude Code remote (web) environment
if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
  exit 0
fi

# Start Docker daemon if not running
"$CLAUDE_PROJECT_DIR/scripts/devcontainer-start-docker.sh"

# Install Bun if not available
if ! command -v bun >/dev/null 2>&1; then
  echo "Installing Bun..."
  curl -fsSL https://bun.com/install | bash
fi

export PATH="$HOME/.bun/bin:$PATH"

# Install devcontainer CLI if not available
if ! command -v devcontainer >/dev/null 2>&1; then
  echo "Installing @devcontainers/cli..."
  bun add -g @devcontainers/cli
fi

# Bring up the devcontainer (runs all lifecycle hooks)
devcontainer up --workspace-folder "$CLAUDE_PROJECT_DIR"
