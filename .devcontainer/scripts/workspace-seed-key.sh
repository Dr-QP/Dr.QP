#!/usr/bin/env bash
set -euo pipefail

# Runs on the host: macOS ships `shasum` but not `sha256sum` without coreutils.
if command -v sha256sum >/dev/null 2>&1; then
    hasher=(sha256sum)
elif command -v shasum >/dev/null 2>&1; then
    hasher=(shasum -a 256)
else
    echo "workspace-seed-key: neither sha256sum nor shasum is available" >&2
    exit 1
fi

digest="$(printf '%s' "${1:?workspace path is required}" | "${hasher[@]}")"
printf '%s\n' "${digest:0:16}"
