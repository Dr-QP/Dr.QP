#!/usr/bin/env bash
# Instance-local D-Bus session and GNOME Keyring Secret Service. Sourced, it provides
# the shared keyring primitives; executed, it runs one supervised piece in the foreground:
# runtime-keyring.sh env|dbus|keyring|check|wait [seconds]

# The login keyring is created with an empty password, so its secrets are stored
# unencrypted; see the AT-REST STORAGE TRADEOFF note in setup-keyring.sh.
keyring_dir="$HOME/.local/share/keyrings"
login_keyring="$keyring_dir/login.keyring"

keyring_session_ready() {
  [[ -n ${DBUS_SESSION_BUS_ADDRESS:-} ]] || return 1
  gdbus call --session \
    --dest org.freedesktop.DBus \
    --object-path /org/freedesktop/DBus \
    --method org.freedesktop.DBus.GetNameOwner \
    org.freedesktop.secrets >/dev/null 2>&1
}

# Never overwrite an existing login.keyring: it is the secret store itself.
keyring_ensure_login_keyring() {
  mkdir -p "$keyring_dir"
  if [[ -f $login_keyring ]]; then
    return 0
  fi
  cat >"$login_keyring" <<EOF
[keyring]
display-name=login
ctime=$(date +%s)
mtime=0
lock-on-idle=false
lock-after=false
EOF
  chmod 600 "$login_keyring"
}

keyring_wait_ready() {
  local timeout_s="${1:-5}" deadline
  deadline=$((SECONDS + timeout_s))
  until keyring_session_ready; do
    if ((SECONDS >= deadline)); then
      echo "Timeout waiting for org.freedesktop.secrets to become available" >&2
      return 1
    fi
    sleep 0.1
  done
}

if [[ ${BASH_SOURCE[0]} != "$0" ]]; then
  return 0
fi

set -euo pipefail

if [[ -z ${XDG_RUNTIME_DIR:-} ]]; then
  echo "runtime-keyring: XDG_RUNTIME_DIR is required" >&2
  exit 1
fi
bus_socket="$XDG_RUNTIME_DIR/bus"
export DBUS_SESSION_BUS_ADDRESS="unix:path=$bus_socket"

wait_for_bus() {
  local deadline=$((SECONDS + ${AGENTDEV_SERVICE_START_TIMEOUT:-60}))
  until gdbus call --session --dest org.freedesktop.DBus --object-path /org/freedesktop/DBus \
    --method org.freedesktop.DBus.GetId >/dev/null 2>&1; do
    if ((SECONDS >= deadline)); then
      echo "runtime-keyring: D-Bus session bus at $bus_socket is not available" >&2
      return 1
    fi
    sleep 0.1
  done
}

case "${1:-}" in
  env)
    printf "export XDG_RUNTIME_DIR='%s'\n" "$XDG_RUNTIME_DIR"
    printf "export DBUS_SESSION_BUS_ADDRESS='%s'\n" "$DBUS_SESSION_BUS_ADDRESS"
    ;;
  dbus)
    mkdir -p "$XDG_RUNTIME_DIR"
    chmod 700 "$XDG_RUNTIME_DIR"
    # A socket left by a killed daemon would make the bind fail.
    rm -f "$bus_socket"
    exec dbus-daemon --session --nofork --nopidfile --address="$DBUS_SESSION_BUS_ADDRESS"
    ;;
  keyring)
    wait_for_bus
    keyring_ensure_login_keyring
    exec gnome-keyring-daemon --foreground --components=secrets
    ;;
  check) keyring_session_ready ;;
  wait) keyring_wait_ready "${2:-${AGENTDEV_SERVICE_START_TIMEOUT:-60}}" ;;
  *)
    echo "usage: $(basename "$0") env|dbus|keyring|check|wait [seconds]" >&2
    exit 2
    ;;
esac
