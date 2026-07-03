#!/usr/bin/env bash

# -e should not be to allow retry script to continue on test failure
set -uo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root_dir=$(dirname "$script_dir")
cd "$root_dir"

max_attempts="${COLCON_TEST_MAX_ATTEMPTS:-3}"
attempt=1

ORIG_PYTEST_ADDOPTS="${PYTEST_ADDOPTS:-}"

while true; do
  if [ "$attempt" -eq 1 ]; then
    python3 -m colcon test --return-code-on-test-failure "$@"
  else
    echo "::group::colcon test retry $attempt/$max_attempts (packages-select-test-failures, pytest --lf)"
    PYTEST_ADDOPTS="${ORIG_PYTEST_ADDOPTS:-} --lf" python3 -m colcon test \
      --return-code-on-test-failure --packages-select-test-failures "$@"
    echo "::endgroup::"
  fi
  status=$?

  if [ "$status" -eq 0 ]; then
    exit 0
  fi

  if [ "$attempt" -ge "$max_attempts" ]; then
    echo "colcon test still failing after $attempt attempt(s), exit code $status" >&2
    while IFS= read -r pkg_dir; do
      python3 -m colcon test-result \
        --test-result-base "build/$(basename "$pkg_dir")" --all --verbose
    done < <(find log/latest_test -mindepth 1 -maxdepth 1 -type d 2>/dev/null)
    exit "$status"
  fi

  attempt=$((attempt + 1))
done
