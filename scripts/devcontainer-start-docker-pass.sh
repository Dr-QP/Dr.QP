#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "$script_dir/.." && pwd)"
env_file="$workspace_dir/.tmp/docker-pass-session.env"

load_session_env() {
    if [[ -f "$env_file" ]]; then
        # shellcheck disable=SC1090
        source "$env_file"
    fi
}

session_ready() {
    load_session_env
    [[ -n "${DBUS_SESSION_BUS_ADDRESS:-}" ]] || return 1
    gdbus call --session \
        --dest org.freedesktop.DBus \
        --object-path /org/freedesktop/DBus \
        --method org.freedesktop.DBus.GetNameOwner \
        org.freedesktop.secrets >/dev/null 2>&1
}

write_env_file() {
    mkdir -p "$workspace_dir/.tmp"
    cat >"$env_file" <<EOF
export DBUS_SESSION_BUS_ADDRESS='${DBUS_SESSION_BUS_ADDRESS}'
export DBUS_SESSION_BUS_PID='${DBUS_SESSION_BUS_PID:-}'
export GNOME_KEYRING_CONTROL='${GNOME_KEYRING_CONTROL:-}'
export SSH_AUTH_SOCK='${SSH_AUTH_SOCK:-}'
EOF
    chmod 600 "$env_file"
}

if session_ready; then
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
    export GNOME_KEYRING_CONTROL SSH_AUTH_SOCK
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
