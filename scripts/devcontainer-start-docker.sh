#!/usr/bin/env bash
set -euo pipefail

docker_host="${DOCKER_HOST:-unix:///var/run/docker.sock}"
docker_log_file="/tmp/dockerd.log"
docker_data_root="/var/lib/docker"

is_overlay_backed() {
  local path="$1"
  [[ -d "$path" ]] && [[ "$(stat -f -c %T "$path")" == "overlayfs" ]]
}

docker_socket_path() {
  if [[ "$docker_host" =~ ^unix://(.+)$ ]]; then
    printf '%s\n' "${BASH_REMATCH[1]}"
  fi
}

is_dockerd_pid() {
  local pid="$1"
  [[ -n "$pid" ]] && ps -p "$pid" -o comm= 2>/dev/null | grep -qx 'dockerd'
}

dockerd_pid_from_socket() {
  local socket_path="$1"
  local pid

  [[ -S "$socket_path" ]] || return 1
  command -v ss >/dev/null 2>&1 || return 1

  pid="$(ss -xlpnH 2>/dev/null | awk -v socket_path="$socket_path" '
    index($0, socket_path) {
      if (match($0, /pid=[0-9]+/)) {
        print substr($0, RSTART + 4, RLENGTH - 4)
        exit
      }
    }
  ' || true)"
  if is_dockerd_pid "$pid"; then
    printf '%s\n' "$pid"
    return 0
  fi

  return 1
}

local_dockerd_pid() {
  local socket_path pid

  socket_path="$(docker_socket_path || true)"
  if [[ -n "$socket_path" ]]; then
    pid="$(dockerd_pid_from_socket "$socket_path" || true)"
    if [[ -n "$pid" ]]; then
      printf '%s\n' "$pid"
      return 0
    fi
  fi

  for pidfile in /tmp/dockerd-vfs.pid /var/run/docker.pid; do
    if [[ -r "$pidfile" ]]; then
      pid="$(cat "$pidfile" 2>/dev/null || true)"
      if is_dockerd_pid "$pid"; then
        printf '%s\n' "$pid"
        return 0
      fi
    fi
  done

  return 1
}

if ! command -v docker >/dev/null 2>&1; then
  echo "docker CLI not found; rebuild the devcontainer image." >&2
  exit 1
fi

if ! command -v dockerd >/dev/null 2>&1; then
  echo "dockerd not found; rebuild the devcontainer image." >&2
  exit 1
fi

if docker info >/dev/null 2>&1; then
  existing_local_dockerd_pid="$(local_dockerd_pid)"
  if [[ -z "$existing_local_dockerd_pid" ]]; then
    exit 0
  fi

  existing_driver="$(docker info --format '{{.Driver}}' 2>/dev/null || true)"
  existing_root_dir="$(docker info --format '{{.DockerRootDir}}' 2>/dev/null || true)"

  if [[ "$existing_driver" == "overlayfs" ]] && is_overlay_backed "$existing_root_dir"; then
    echo "Detected running dockerd using overlayfs on overlayfs; restarting with vfs."
    kill "$existing_local_dockerd_pid"
    for _ in $(seq 1 15); do
      if ! docker info >/dev/null 2>&1; then
        break
      fi
      sleep 1
    done
  else
    exit 0
  fi
fi

dockerd_args=(
  -H "$docker_host"
)

docker_data_root="/var/lib/docker"

if [[ -d "$docker_data_root" ]] && [[ "$(stat -f -c %T "$docker_data_root")" == "overlayfs" ]]; then
  echo "Detected overlayfs for $docker_data_root; starting dockerd with vfs storage driver."
  dockerd_args+=(
    --storage-driver=vfs
    --data-root /tmp/docker-vfs
    --exec-root /tmp/docker-vfs-exec
    --pidfile /tmp/dockerd-vfs.pid
    --iptables=false
  )
  docker_log_file="/tmp/dockerd-vfs.log"
fi

dockerd "${dockerd_args[@]}" >"$docker_log_file" 2>&1 &

for _ in $(seq 1 30); do
  if docker info >/dev/null 2>&1; then
    exit 0
  fi
  sleep 1
done

echo "Failed to start dockerd (DOCKER_HOST=$docker_host)." >&2
tail -n 50 "$docker_log_file" || true
exit 1
