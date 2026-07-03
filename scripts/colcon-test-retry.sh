#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root_dir=$(dirname "$script_dir")
cd "$root_dir"

max_attempts="${COLCON_TEST_MAX_ATTEMPTS:-3}"
attempt=1

ORIG_PYTEST_ADDOPTS="${PYTEST_ADDOPTS:-}"

while true; do
  if [ "$attempt" -eq 1 ]; then
    python3 -m colcon test "$@"
  else
    echo "::group::colcon test retry $attempt/$max_attempts (packages-select-test-failures)"
    env PYTEST_ADDOPTS="${ORIG_PYTEST_ADDOPTS} --lf" python3 -m colcon test --packages-select-test-failures "$@"
    echo "::endgroup::"
  fi

  if python3 -m colcon test-result --all; then
    exit 0
  fi

  if [ "$attempt" -ge "$max_attempts" ]; then
    echo "colcon test still failing after $attempt attempt(s):" >&2
    python3 -m colcon test-result --all --verbose
    exit 1
  fi

  attempt=$((attempt + 1))
done
