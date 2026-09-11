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

codebase-memory-mcp daemon status

codebase-memory-mcp cli index_repository --repo-path "$DEV_WORKSPACE_FOLDER"
