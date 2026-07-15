#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
root_dir=$(cd "$script_dir/../.." && pwd)
. "$script_dir/super-linter-defaults.sh"

image="${SUPER_LINTER_IMAGE:-$SUPER_LINTER_DEFAULT_IMAGE}"

exec "$runtime" run --rm \
  -e RUN_LOCAL=true \
  -v "$root_dir:/tmp/lint" \
  --platform linux/amd64 \
  --entrypoint /bin/bash \
  "$image" \
  -c 'exec "$@"' _ "$@"
