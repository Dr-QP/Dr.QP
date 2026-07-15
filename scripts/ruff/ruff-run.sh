#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
root_dir=$(cd "$script_dir/../.." && pwd)
dockerfile="$root_dir/docker/ruff/Dockerfile"

image="${RUFF_IMAGE:-drqp-ruff:0.15.0-alpine-arm64}"
container_name="${RUFF_CONTAINER_NAME:-ruff-run}"

if command -v docker >/dev/null 2>&1; then
  runtime=docker
elif command -v podman >/dev/null 2>&1; then
  runtime=podman
else
  echo "Docker or Podman is required to run Ruff." >&2
  exit 1
fi

image_is_native_arm64() {
  local architecture
  architecture=$("$runtime" image inspect --format '{{.Architecture}}' "$image" \
    2>/dev/null) || return 1
  [[ "$architecture" == "arm64" || "$architecture" == "aarch64" ]]
}

build_image() {
  "$runtime" build \
    --platform linux/arm64 \
    --file "$dockerfile" \
    --tag "$image" \
    "$root_dir/docker/ruff"
}

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
    -v "$root_dir:$root_dir" \
    -w "$root_dir" \
    --platform linux/arm64 \
    --entrypoint /bin/sh \
    "$image" \
    -c 'while :; do sleep 3600; done' >/dev/null
}

if ! image_is_native_arm64; then
  build_image
fi

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
  ruff "$@"
