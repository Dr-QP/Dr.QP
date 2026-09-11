#!/usr/bin/env bash
set -xeuo pipefail

vhclient=${VIRTUALHERE_CLIENT_BIN:-/usr/sbin/vhclient}
manual_hub=${VIRTUALHERE_MANUAL_HUB:-host.docker.internal:7575}

if [[ ! -x "$vhclient" ]]; then
    exit 0
fi

if ! "$vhclient" -t "LIST" >/dev/null 2>&1; then
    if ! "$vhclient" --daemon >/dev/null 2>&1; then
        echo "VirtualHere client did not start; skipping manual hub setup" >&2
        exit 0
    fi
fi

if ! "$vhclient" -t "MANUAL HUB ADD,${manual_hub}" >/dev/null 2>&1; then
    echo "VirtualHere manual hub setup failed for ${manual_hub}" >&2
fi

sleep 1

DEVICE_NAME="DualSense Wireless Controller"
DEVICE_ADDR="$(
  "$vhclient" -t "LIST" \
    | awk -v device="$DEVICE_NAME" '
        $0 ~ device && match($0, /\(host\.[0-9]+\)/) {
          print substr($0, RSTART + 1, RLENGTH - 2)
          exit
        }
      '
)"

if [[ -z "$DEVICE_ADDR" ]]; then
    echo "VirtualHere \"$DEVICE_NAME\" not found; skipping device use"
    exit 0
fi

echo "VirtualHere \"$DEVICE_NAME\" address: ${DEVICE_ADDR}"
"$vhclient" -t "USE,$DEVICE_ADDR"
