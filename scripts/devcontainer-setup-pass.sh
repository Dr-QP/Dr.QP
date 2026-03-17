#!/usr/bin/env bash
# Starts the Docker Pass Secret Service runtime inside the devcontainer and imports
# any docker/mcp/ secrets that were exported to .tmp/docker-pass-export by
# devcontainer-init.sh on the host. When a live Docker secrets engine socket is
# mounted into the container, that engine remains the runtime source of truth and
# no local secret mirroring is required during startup.
set -xeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "$script_dir/.." && pwd)"

EXPORT_FILE="$workspace_dir/.tmp/docker-pass-export"
DOCKER_PASS_ENV_FILE="$workspace_dir/.tmp/docker-pass-session.env"
DOCKER_PASS_EXTERNAL_ENGINE="${DOCKER_PASS_EXTERNAL_ENGINE:-0}"

list_docker_mcp_secrets() {
    docker mcp secret ls 2>/dev/null | awk -F '|' '
        {
            name = $1
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", name)
            if (name ~ /^docker\/mcp\//) {
                print name
            }
        }
    '
}

stream_live_engine_secrets() {
    local args=()
    local name=""
    local print_cmd=""
    local secret_count=0
    local index

    while IFS= read -r name; do
        [[ -z "$name" ]] && continue
        args+=(-e "_N${secret_count}=${name}" -e "_S${secret_count}=se://${name}")
        secret_count=$((secret_count + 1))
    done < <(list_docker_mcp_secrets)

    if (( secret_count == 0 )); then
        return 1
    fi

    for index in $(seq 0 $((secret_count - 1))); do
        print_cmd+="[ -n \"\$_S${index}\" ] && printf '%s\t%s\n' \"\$_N${index}\" \"\$_S${index}\"; "
    done

    docker run --rm "${args[@]}" busybox sh -c "$print_cmd" 2>/dev/null
}

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

import_secret_stream() {
    local imported_count=0
    local name=""
    local value=""

    while IFS=$'\t' read -r name value; do
        [[ -z "$name" ]] && continue
        import_secret "$name" "$value"
        imported_count=$((imported_count + 1))
    done

    if (( imported_count == 0 )); then
        return 1
    fi

    return 0
}

with_secret_tracing_disabled() {
    local xtrace_was_enabled=0

    if [[ $- == *x* ]]; then
        xtrace_was_enabled=1
        set +x
    fi

    "$@"
    local status=$?

    if [[ $xtrace_was_enabled -eq 1 ]]; then
        set -x
    fi

    return "$status"
}

import_exported_secrets() {
    echo "Importing docker/mcp/ secrets from $EXPORT_FILE..."
    with_secret_tracing_disabled import_secret_stream < <(
        openssl enc -aes-256-cbc -d -pbkdf2 \
            -pass "pass:${DOCKER_PASS_EXPORT_KEY}" -in "$EXPORT_FILE" 2>/dev/null
    )
    rm -f "$EXPORT_FILE"
    echo "Secrets imported and export file removed."
}

import_live_engine_secrets() {
    if ! list_docker_mcp_secrets | grep -q .; then
        echo "No docker/mcp/ secrets available from the active Docker secrets engine; skipping import."
        return 0
    fi

    echo "Importing docker/mcp/ secrets from the active Docker secrets engine..."
    with_secret_tracing_disabled import_secret_stream < <(stream_live_engine_secrets)
    echo "Secrets imported from active Docker secrets engine."
}

"$script_dir/devcontainer-start-docker-pass.sh"

if [[ -f "$DOCKER_PASS_ENV_FILE" ]]; then
    # shellcheck disable=SC1090
    source "$DOCKER_PASS_ENV_FILE"
fi

# Import secrets exported by devcontainer-init.sh (AES-encrypted), then delete the file.
if [[ -f "$EXPORT_FILE" ]] && [[ -n "${DOCKER_PASS_EXPORT_KEY:-}" ]]; then
    import_exported_secrets
elif [[ "$DOCKER_PASS_EXTERNAL_ENGINE" == "1" ]]; then
    echo "Using mounted Docker secrets engine at ${DOCKER_SECRETS_ENGINE_SOCKET:-$HOME/.cache/docker-secrets-engine/engine.sock}; skipping local secret import."
else
    echo "No docker-pass export file or decryption key; skipping secret import."
fi
