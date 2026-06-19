#!/usr/bin/env bash
# Bring up a GNOME Keyring Secret Service (org.freedesktop.secrets) inside the
# container. Safe to run standalone to enable the keyring, and reused by
# devcontainer-start-docker-pass.sh.
#
# Idempotent: a live session is reused, a saved session is restored, otherwise
# a fresh dbus + gnome-keyring session is started. The resulting D-Bus session
# environment is persisted to .tmp/keyring-session.env so other processes
# (docker-pass engine, gh, etc.) can source it and reach the keyring.
#
# IMPORTANT: login.keyring is the actual secret store. Overwriting it destroys
# every secret already saved in it (e.g. `gh auth login` tokens), so we must
# NEVER clobber an existing keyring.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "$script_dir/.." && pwd)"
env_file="$workspace_dir/.tmp/keyring-session.env"
keyring_dir="$HOME/.local/share/keyrings"
login_keyring="$keyring_dir/login.keyring"

session_ready() {
    [[ -n "${DBUS_SESSION_BUS_ADDRESS:-}" ]] || return 1
    gdbus call --session \
        --dest org.freedesktop.DBus \
        --object-path /org/freedesktop/DBus \
        --method org.freedesktop.DBus.GetNameOwner \
        org.freedesktop.secrets >/dev/null 2>&1
}

load_saved_session() {
    [[ -f "$env_file" ]] || return 1
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

ensure_login_keyring() {
    mkdir -p "$keyring_dir"

    # Preserve any keyring that already holds secrets.
    if [[ -f "$login_keyring" ]]; then
        return 0
    fi

    cat >"$login_keyring" <<'EOF'
[keyring]
display-name=login
ctime=1750965549
mtime=0
lock-on-idle=false
lock-after=false
EOF
    chmod 600 "$login_keyring"
}

require_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "$1 is not installed" >&2
        exit 1
    fi
}

# Reuse a live Secret Service session, or restore a previously-saved one.
if session_ready; then
    write_env_file
    exit 0
fi

if load_saved_session; then
    write_env_file
    exit 0
fi

rm -f "$env_file"
unset DBUS_SESSION_BUS_ADDRESS DBUS_SESSION_BUS_PID GNOME_KEYRING_CONTROL

require_command dbus-daemon
require_command gnome-keyring-daemon
require_command gdbus

ensure_login_keyring

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
        echo "Timeout waiting for org.freedesktop.secrets to become available" >&2
        exit 1
    fi
done

write_env_file
