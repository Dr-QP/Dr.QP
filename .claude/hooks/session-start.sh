#!/bin/bash
set -euo pipefail

# Only run in Claude Code remote (web) environment
if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
  exit 0
fi

# Start Docker daemon if not running
"$CLAUDE_PROJECT_DIR/scripts/devcontainer-start-docker.sh"

# Install devcontainer CLI if not available
if ! command -v devcontainer >/dev/null 2>&1; then
  echo "Installing @devcontainers/cli..."
  npm install -g @devcontainers/cli
fi

# Bring up the devcontainer (runs all lifecycle hooks)
devcontainer up --workspace-folder "$CLAUDE_PROJECT_DIR"
