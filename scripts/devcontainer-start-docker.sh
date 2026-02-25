#!/usr/bin/env bash
set -euo pipefail

docker_host="${DOCKER_HOST:-unix:///var/run/docker.sock}"
docker_log_file="/tmp/dockerd.log"
docker_data_root="/var/lib/docker"
docker_startup_retries=30
docker_shutdown_retries=15
dockerd_args=(-H "$docker_host")

docker_info() {
  docker info >/dev/null 2>&1
}

die() {
  echo "$1" >&2
  exit 1
}

has_command() {
  command -v "$1" >/dev/null 2>&1
}

is_overlay_backed() {
  local path="$1"
  [[ -d "$path" ]] && [[ "$(stat -f -c %T "$path")" == "overlayfs" ]]
}

docker_driver() {
  docker info --format '{{.Driver}}' 2>/dev/null || true
}

docker_root_dir() {
  docker info --format '{{.DockerRootDir}}' 2>/dev/null || true
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

wait_for_docker_up() {
  local _
  for _ in $(seq 1 "$docker_startup_retries"); do
    if docker_info; then
      return 0
    fi
    sleep 1
  done

  return 1
}

wait_for_docker_down() {
  local _
  for _ in $(seq 1 "$docker_shutdown_retries"); do
    if ! docker_info; then
      return 0
    fi
    sleep 1
  done

  return 1
}

running_daemon_needs_vfs_restart() {
  local existing_driver existing_root_dir

  existing_driver="$(docker_driver)"
  existing_root_dir="$(docker_root_dir)"
  [[ "$existing_driver" == "overlay2" || "$existing_driver" == "overlay" ]] &&
    is_overlay_backed "$existing_root_dir"
}

configure_storage_driver_args() {
  if is_overlay_backed "$docker_data_root"; then
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
}

restart_local_daemon_if_needed() {
  local existing_local_dockerd_pid

  if ! docker_info; then
    return 0
  fi

  existing_local_dockerd_pid="$(local_dockerd_pid || true)"
  if [[ -z "$existing_local_dockerd_pid" ]]; then
    exit 0
  fi

  if ! running_daemon_needs_vfs_restart; then
    exit 0
  fi

  echo "Detected running dockerd using overlayfs on overlayfs; restarting with vfs."
  kill -TERM "$existing_local_dockerd_pid" || true
  if ! wait_for_docker_down; then
    echo "dockerd did not stop after SIGTERM; sending SIGKILL." >&2
    kill -KILL "$existing_local_dockerd_pid" || true
    if ! wait_for_docker_down; then
      die "Failed to stop existing dockerd process (pid=$existing_local_dockerd_pid); aborting restart."
    fi
  fi
}

start_dockerd() {
  dockerd "${dockerd_args[@]}" >"$docker_log_file" 2>&1 &
}

validate_prerequisites() {
  has_command docker || die "docker CLI not found; rebuild the devcontainer image."
  has_command dockerd || die "dockerd not found; rebuild the devcontainer image."
}

main() {
  validate_prerequisites
  restart_local_daemon_if_needed
  configure_storage_driver_args
  start_dockerd

  if wait_for_docker_up; then
    exit 0
  fi

  echo "Failed to start dockerd (DOCKER_HOST=$docker_host)." >&2
  tail -n 50 "$docker_log_file" || true
  exit 1
}

main
