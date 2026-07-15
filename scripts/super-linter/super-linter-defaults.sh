#!/usr/bin/env bash

# shellcheck disable=SC2034 # Sourced by Super-Linter wrapper scripts.
SUPER_LINTER_DEFAULT_VERSION="v8.5.0"

# shellcheck disable=SC2034 # Sourced by Super-Linter wrapper scripts.
SUPER_LINTER_DEFAULT_IMAGE="ghcr.io/super-linter/super-linter:${SUPER_LINTER_DEFAULT_VERSION}"

if command -v docker >/dev/null 2>&1; then
  runtime=docker
elif command -v podman >/dev/null 2>&1; then
  runtime=podman
else
  echo "Docker or Podman is required to inspect Super-Linter." >&2
  exit 1
fi
