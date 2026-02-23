#!/usr/bin/env bash
set -euo pipefail

docker_host="${DOCKER_HOST:-unix:///var/run/docker.sock}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker CLI not found; rebuild the devcontainer image." >&2
  exit 1
fi

if ! command -v dockerd >/dev/null 2>&1; then
  echo "dockerd not found; rebuild the devcontainer image." >&2
  exit 1
fi

if docker info >/dev/null 2>&1; then
  exit 0
fi

if pgrep -x dockerd >/dev/null 2>&1; then
  for _ in $(seq 1 20); do
    if docker info >/dev/null 2>&1; then
      exit 0
    fi
    sleep 1
  done
  echo "dockerd appears to be running but is not responding after 20 seconds; refusing to start a second dockerd instance." >&2
  exit 1
fi

dockerd -H "$docker_host" >/tmp/dockerd.log 2>&1 &

for _ in $(seq 1 30); do
  if docker info >/dev/null 2>&1; then
    exit 0
  fi
  sleep 1
done

echo "Failed to start dockerd (DOCKER_HOST=$docker_host)." >&2
tail -n 50 /tmp/dockerd.log || true
exit 1
