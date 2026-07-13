#!/usr/bin/env bash

set -euo pipefail

mkdir -p ./.tmp
findings_diff=$(mktemp ./.tmp/shellcheck-fix.XXXXXX.diff)
trap 'rm -f -- "$findings_diff"' EXIT

mapfile -d '' scripts < <(
  find . \
    -path '*/vendor' -prune -o \
    -path '*/drqp_rapidjson/include' -prune -o \
    -iname '*.sh' \
    -printf '%P\0'
)

if ((${#scripts[@]} == 0)); then
  exit 0
fi

set +e
shellcheck -f diff "${scripts[@]}" >"$findings_diff"
shellcheck_status=$?
set -e

if ((shellcheck_status > 1)); then
  exit "$shellcheck_status"
fi

if [[ -s "$findings_diff" ]]; then
  git apply "$findings_diff"
fi
