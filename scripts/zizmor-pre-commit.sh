#!/usr/bin/env bash

set -euo pipefail

ZIZMOR_VERSION="1.22.0"

if ! command -v uvx >/dev/null 2>&1; then
  echo "uvx is required to run the pinned zizmor pre-commit hook." >&2
  exit 1
fi

exec uvx --from "zizmor==${ZIZMOR_VERSION}" zizmor "$@"
