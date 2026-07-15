#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
root_dir=$(cd "$script_dir/../.." && pwd)
. "$script_dir/super-linter-defaults.sh"

image="${SUPER_LINTER_IMAGE:-$SUPER_LINTER_DEFAULT_IMAGE}"
container_name="${SUPER_LINTER_CONTAINER_NAME:-super-linter-run}"

container_exists() {
  "$runtime" container inspect "$container_name" >/dev/null 2>&1
}

container_is_running() {
  [[ $("$runtime" container inspect --format '{{.State.Running}}' \
    "$container_name") == "true" ]]
}

container_uses_requested_image() {
  [[ $("$runtime" container inspect --format '{{.Config.Image}}' \
    "$container_name") == "$image" ]]
}

start_container() {
  "$runtime" run --detach \
    --name "$container_name" \
    -e RUN_LOCAL=true \
    -v "$root_dir:$root_dir" \
    -w "$root_dir" \
    --platform linux/amd64 \
    --entrypoint /bin/bash \
    "$image" \
    -c 'while :; do sleep 3600; done' >/dev/null
}

if container_exists; then
  if ! container_is_running || ! container_uses_requested_image; then
    "$runtime" container rm --force "$container_name" >/dev/null
    start_container
  fi
else
  start_container
fi

exec "$runtime" exec \
  -w "$root_dir" \
  "$container_name" \
  /bin/bash -c 'exec "$@"' _ "$@"
