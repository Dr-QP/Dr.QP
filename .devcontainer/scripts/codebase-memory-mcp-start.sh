#!/usr/bin/env bash
set -euo pipefail

if ! command -v codebase-memory-mcp &>/dev/null; then
  echo "codebase-memory-mcp is not installed; skipping agent config wiring."
  exit 0
fi

if [[ -z $CBM_CACHE_DIR ]]; then
  echo "CBM_CACHE_DIR is required for codebase-memory-mcp install; it is not set in this container." >&2
  exit 1
fi

# Keep the codebase-memory-mcp daemon running in the background so that the
# CLI can talk to it without needing to start a new daemon on every invocation.
# Environment used by daemon-owned components—such as diagnostics, daemon logging, and process-wide indexing resource limits—is captured from the first daemon-backed session that starts the daemon. Later sessions join that process and cannot replace those values. To change them, close all daemon-backed sessions, update the relevant agent configurations consistently, and restart a session. CBM_ALLOWED_ROOT remains session-specific, a conflicting CBM_CACHE_DIR is rejected, and one-shot CLI commands read their own environment without starting the daemon.
codebase-memory-mcp daemon start

codebase-memory-mcp cli index_repository --repo-path "$DEV_WORKSPACE_FOLDER"
