#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "$script_dir/.." && pwd)"
env_file="$workspace_dir/.tmp/docker-pass-session.env"
cache_dir="${XDG_CACHE_HOME:-$HOME/.cache}/docker-secrets-engine"
engine_socket="${DOCKER_SECRETS_ENGINE_SOCKET:-$cache_dir/engine.sock}"
engine_pid_file="$workspace_dir/.tmp/docker-pass-engine.pid"
engine_log_file="$workspace_dir/.tmp/docker-pass-engine.log"
engine_binary="$HOME/.docker/cli-plugins/docker-pass"
external_engine="${DOCKER_PASS_EXTERNAL_ENGINE:-0}"

session_ready() {
    [[ -n "${DBUS_SESSION_BUS_ADDRESS:-}" ]] || return 1
    gdbus call --session \
        --dest org.freedesktop.DBus \
        --object-path /org/freedesktop/DBus \
        --method org.freedesktop.DBus.GetNameOwner \
        org.freedesktop.secrets >/dev/null 2>&1
}

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

    if [[ ! -x "$engine_binary" ]]; then
        return 0
    fi

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

load_saved_session() {
    if [[ ! -f "$env_file" ]]; then
        return 1
    fi

    # shellcheck disable=SC1090
    source "$env_file"
    session_ready
}

write_env_file() {
    mkdir -p "$workspace_dir/.tmp"
    cat >"$env_file" <<EOF
export DBUS_SESSION_BUS_ADDRESS='${DBUS_SESSION_BUS_ADDRESS}'
export DBUS_SESSION_BUS_PID='${DBUS_SESSION_BUS_PID:-}'
export GNOME_KEYRING_CONTROL='${GNOME_KEYRING_CONTROL:-}'
EOF
    chmod 600 "$env_file"
}

if session_ready; then
    write_env_file
    ensure_local_engine
    exit 0
fi

if load_saved_session; then
    ensure_local_engine
    exit 0
fi

rm -f "$env_file"
unset DBUS_SESSION_BUS_ADDRESS DBUS_SESSION_BUS_PID GNOME_KEYRING_CONTROL

if [[ ! -x "$engine_binary" ]]; then
    echo "docker-pass plugin not installed; skipping Secret Service setup."
    exit 0
fi

if ! command -v dbus-daemon >/dev/null 2>&1; then
    echo "dbus-daemon is not installed"
    exit 1
fi

if ! command -v gnome-keyring-daemon >/dev/null 2>&1; then
    echo "gnome-keyring-daemon is not installed"
    exit 1
fi

if ! command -v gdbus >/dev/null 2>&1; then
    echo "gdbus is not installed"
    exit 1
fi

mkdir -p ~/.local/share/keyrings

cat > ~/.local/share/keyrings/login.keyring <<'EOF'
[keyring]
display-name=login
ctime=1750965549
mtime=0
lock-on-idle=false
lock-after=false
EOF

dbus_output="$(dbus-daemon --session --fork --print-address=1 --print-pid=1)"
DBUS_SESSION_BUS_ADDRESS="$(printf '%s\n' "$dbus_output" | sed -n '1p')"
DBUS_SESSION_BUS_PID="$(printf '%s\n' "$dbus_output" | sed -n '2p')"
export DBUS_SESSION_BUS_ADDRESS DBUS_SESSION_BUS_PID

keyring_output="$(gnome-keyring-daemon --start --components=secrets)"
if [[ -n "$keyring_output" ]]; then
    eval "$keyring_output"
    export GNOME_KEYRING_CONTROL
fi

timeout_ms=5000
interval_ms=100
elapsed_ms=0

while ! session_ready; do
    sleep 0.1
    elapsed_ms=$((elapsed_ms + interval_ms))
    if (( elapsed_ms >= timeout_ms )); then
        echo "Timeout waiting for org.freedesktop.secrets to become available"
        exit 1
    fi
done

write_env_file
ensure_local_engine
