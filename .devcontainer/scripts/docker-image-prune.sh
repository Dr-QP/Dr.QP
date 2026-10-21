#!/usr/bin/env bash
# Removes unused Dev Container engine images created over 72 hours ago; see
# spec/devcontainer-docker-storage.

set -euo pipefail

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is not installed; skipping image prune."
  exit 0
fi

if ! docker info >/dev/null 2>&1; then
  echo "The Docker daemon is unreachable; skipping image prune."
  exit 0
fi

if ! output="$(docker image prune -af --filter until=72h 2>&1)"; then
  printf '%s\n' "$output" >&2
  echo "Warning: docker image prune failed; continuing." >&2
  exit 0
fi

while IFS= read -r line; do
  if [[ "$line" == "Total reclaimed space:"* ]]; then
    echo "$line"
  fi
done <<<"$output"
