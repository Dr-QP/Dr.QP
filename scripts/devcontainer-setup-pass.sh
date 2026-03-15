#!/usr/bin/env bash
# Starts the Docker Pass Secret Service runtime inside the devcontainer and imports
# any docker/mcp/ secrets that were exported to .tmp/docker-pass-export by
# devcontainer-init.sh on the host.
set -xeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "$script_dir/.." && pwd)"

EXPORT_FILE="$workspace_dir/.tmp/docker-pass-export"
DOCKER_PASS_ENV_FILE="$workspace_dir/.tmp/docker-pass-session.env"

import_secret() {
    local name="$1"
    local value="$2"
    local attempt=1
    local max_attempts=10
    local error_output=""

    while (( attempt <= max_attempts )); do
        if error_output=$(printf '%s\n' "$value" | docker pass set "$name" 2>&1 >/dev/null); then
            return 0
        fi

        if (( attempt == max_attempts )); then
            echo "$error_output" >&2
            return 1
        fi

        sleep 0.5
        attempt=$((attempt + 1))
    done
}

"$script_dir/devcontainer-start-docker-pass.sh"

if [[ -f "$DOCKER_PASS_ENV_FILE" ]]; then
    # shellcheck disable=SC1090
    source "$DOCKER_PASS_ENV_FILE"
fi

# Import secrets exported by devcontainer-init.sh (AES-encrypted), then delete the file.
if [[ -f "$EXPORT_FILE" ]] && [[ -n "${DOCKER_PASS_EXPORT_KEY:-}" ]]; then
    echo "Importing docker/mcp/ secrets from $EXPORT_FILE..."
    xtrace_was_enabled=0
    if [[ $- == *x* ]]; then
        xtrace_was_enabled=1
        set +x
    fi
    while IFS=$'\t' read -r name value; do
        [[ -z "$name" ]] && continue
        import_secret "$name" "$value"
    done < <(openssl enc -aes-256-cbc -d -pbkdf2 \
        -pass "pass:${DOCKER_PASS_EXPORT_KEY}" -in "$EXPORT_FILE" 2>/dev/null)
    rm -f "$EXPORT_FILE"
    if [[ $xtrace_was_enabled -eq 1 ]]; then
        set -x
    fi
    echo "Secrets imported and export file removed."
else
    echo "No docker-pass export file or decryption key; skipping secret import."
fi
