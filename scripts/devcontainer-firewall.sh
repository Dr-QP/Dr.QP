#!/usr/bin/env bash
set -euo pipefail

if [ "$ENABLE_FIREWALL" != "true" ]; then
    echo "Firewall setup skipped (ENABLE_FIREWALL is not set to 'true')"
    exit 0
fi

if [ ! -x /usr/local/bin/init-firewall.sh ]; then
    echo "WARNING: init-firewall.sh not found in image; skipping firewall setup"
    exit 1
fi

sudo /usr/local/bin/init-firewall.sh
