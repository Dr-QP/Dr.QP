#!/usr/bin/env bash
# Ensure the docker-pass secrets engine is running. The engine stores its
# secrets in the GNOME Keyring Secret Service, so this script first brings up
# the keyring via devcontainer-setup-keyring.sh and loads its D-Bus session.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "$script_dir/.." && pwd)"
keyring_env_file="$workspace_dir/.tmp/keyring-session.env"
cache_dir="${XDG_CACHE_HOME:-$HOME/.cache}/docker-secrets-engine"
engine_socket="${DOCKER_SECRETS_ENGINE_SOCKET:-$cache_dir/engine.sock}"
engine_pid_file="$workspace_dir/.tmp/docker-pass-engine.pid"
engine_log_file="$workspace_dir/.tmp/docker-pass-engine.log"
engine_binary="$HOME/.docker/cli-plugins/docker-pass"
external_engine="${DOCKER_PASS_EXTERNAL_ENGINE:-0}"

engine_ready() {
    [[ -S "$engine_socket" ]] || return 1
    curl --silent --show-error --fail --unix-socket "$engine_socket" \
        http://localhost/health >/dev/null 2>&1
}

wait_for_engine_ready() {
    local timeout_ms="${1:-5000}"
    local interval_ms=100
    local elapsed_ms=0

    while ! engine_ready; do
        sleep 0.1
        elapsed_ms=$((elapsed_ms + interval_ms))
        if (( elapsed_ms >= timeout_ms )); then
            return 1
        fi
    done

    return 0
}

ensure_local_engine() {
    local engine_pid=""
    local timeout_ms=5000

    if engine_ready; then
        return 0
    fi

    if [[ "$external_engine" == "1" ]]; then
        if wait_for_engine_ready "$timeout_ms"; then
            return 0
        fi

        echo "Mounted docker-pass engine socket at $engine_socket is unavailable" >&2
        return 1
    fi

    mkdir -p "$workspace_dir/.tmp"

    if [[ -f "$engine_pid_file" ]]; then
        engine_pid="$(<"$engine_pid_file")"
        if [[ -n "$engine_pid" ]] && kill -0 "$engine_pid" >/dev/null 2>&1; then
            if wait_for_engine_ready "$timeout_ms"; then
                return 0
            fi
        fi
        rm -f "$engine_pid_file"
    fi

    mkdir -p "$cache_dir"
    rm -f "$engine_socket"

    "$engine_binary" internal-engine-serve >"$engine_log_file" 2>&1 &
    engine_pid=$!
    echo "$engine_pid" > "$engine_pid_file"

    while ! engine_ready; do
        if ! kill -0 "$engine_pid" >/dev/null 2>&1; then
            echo "docker-pass local engine exited unexpectedly" >&2
            if [[ -f "$engine_log_file" ]]; then
                cat "$engine_log_file" >&2
            fi
            return 1
        fi

        if ! wait_for_engine_ready "$timeout_ms"; then
            echo "Timeout waiting for docker-pass local engine to become available" >&2
            if [[ -f "$engine_log_file" ]]; then
                cat "$engine_log_file" >&2
            fi
            return 1
        fi
    done
}

if [[ ! -x "$engine_binary" ]]; then
    echo "docker-pass plugin not installed; skipping secrets engine startup."
    exit 0
fi

# The engine talks to the Secret Service, so ensure the keyring is up and
# load its D-Bus session into this process before starting the engine.
"$script_dir/devcontainer-setup-keyring.sh"

if [[ -f "$keyring_env_file" ]]; then
    # shellcheck disable=SC1090
    source "$keyring_env_file"
fi

ensure_local_engine
