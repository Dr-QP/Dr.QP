#!/bin/bash
set -euo pipefail

# Only run in Claude Code remote (web) environment
if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
  exit 0
fi

# Start Docker daemon if not running
if ! docker info >/dev/null 2>&1; then
  echo "Starting Docker daemon..."
  dockerd -H unix:///var/run/docker.sock >/tmp/dockerd.log 2>&1 &
  for i in $(seq 1 30); do
    if docker info >/dev/null 2>&1; then
      echo "Docker daemon is ready."
      break
    fi
    sleep 1
  done
  if ! docker info >/dev/null 2>&1; then
    echo "Failed to start Docker daemon" >&2
    tail -n 20 /tmp/dockerd.log >&2
    exit 1
  fi
fi

# Install devcontainer CLI if not available
if ! command -v devcontainer >/dev/null 2>&1; then
  echo "Installing @devcontainers/cli..."
  npm install -g @devcontainers/cli
fi

# Bring up the devcontainer (runs all lifecycle hooks)
devcontainer up --workspace-folder "$CLAUDE_PROJECT_DIR"
