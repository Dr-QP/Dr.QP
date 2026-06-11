#!/usr/bin/env bash
set -euo pipefail

script_dir=$(dirname "${BASH_SOURCE[0]}")

if [[ -f "$script_dir/local.env" ]]; then
    cp -f "$script_dir/local.env" "$script_dir/.env"
else
    rm -f "$script_dir/.env"
    touch "$script_dir/.env"
fi

gitdir=$(realpath "$(git rev-parse --git-common-dir)")
echo "GIT_REPO=$gitdir" >> "$script_dir/.env"

LOCAL_WORKSPACE_FOLDER=$(realpath "$script_dir/..")
echo "LOCAL_WORKSPACE_FOLDER=$LOCAL_WORKSPACE_FOLDER" >> "$script_dir/.env"

HOST_MCP_DIR="$HOME/.docker/mcp"
CONTAINER_MCP_DIR="/root/.docker/mcp"
if [[ -d "$HOST_MCP_DIR" ]]; then
    echo "HOST_MCP_DIR=$HOST_MCP_DIR" >> "$script_dir/.env"
    echo "CONTAINER_MCP_DIR=$CONTAINER_MCP_DIR" >> "$script_dir/.env"
else
    # Ensure the container MCP dir exists to avoid bind mount errors, even if empty.
    mkdir -p "/tmp/stub-mcp"
fi

# Export docker/mcp/ secrets from the host via `docker pass` so the container can import them.
# The export file is written to .tmp/ (gitignored) and is read+deleted by devcontainer-setup-pass.sh.
_export_docker_pass_secrets() {
    local export_dir="$LOCAL_WORKSPACE_FOLDER/.tmp"
    local export_file="$export_dir/docker-pass-export"

    # Requires the docker-pass CLI plugin (docker/secrets-engine) to be installed on the host.
    if ! docker pass ls >/dev/null 2>&1; then
        return 0
    fi

    # Collect all docker/mcp/ secret names.
    # Pass each as _Ni (name) + _Si (se:// value) so the container can print
    # self-describing "name\tvalue" lines — no index alignment needed on the
    # host side, and `docker pass get` is bypassed (it always masks the value).
    local args=() name i
    i=0
    while IFS= read -r name; do
        args+=(-e "_N${i}=${name}" -e "_S${i}=se://${name}")
        i=$((i + 1))
    done < <(docker pass ls 2>/dev/null | grep '^docker/mcp/')

    if [[ $i -eq 0 ]]; then
        return 0
    fi

    # Build a sh one-liner that prints "name\tvalue" only when the secret was
    # successfully resolved (non-empty); unresolved se:// refs stay empty.
    local print_cmd="" j
    for j in $(seq 0 $((i - 1))); do
        print_cmd+="[ -n \"\$_S${j}\" ] && printf '%s\t%s\n' \"\$_N${j}\" \"\$_S${j}\"; "
    done

    # Encrypt the export file with a per-session random key so it is not stored
    # in plaintext on disk. The key is written to .devcontainer/.env and picked
    # up by docker-compose, then used by devcontainer-setup-pass.sh to decrypt.
    local key
    key=$(openssl rand -hex 32)

    mkdir -p "$export_dir"

    docker run --rm "${args[@]}" busybox sh -c "$print_cmd" 2>/dev/null \
        | openssl enc -aes-256-cbc -pbkdf2 -pass "pass:${key}" -out "$export_file"
    chmod 600 "$export_file"

    echo "DOCKER_PASS_EXPORT_KEY=${key}" >> "$script_dir/.env"
}

HOST_SECRETS_SOCKET="$HOME/Library/Caches/docker-secrets-engine/engine.sock"
DOCKER_SECRETS_ENGINE_SOCKET="/root/.cache/docker-secrets-engine/engine.sock"
if [[ -S "$HOST_SECRETS_SOCKET" ]]; then
    {
        echo "HOST_SECRETS_SOCKET=$HOST_SECRETS_SOCKET"
        echo "DOCKER_SECRETS_ENGINE_SOCKET=$DOCKER_SECRETS_ENGINE_SOCKET"
        echo "DOCKER_PASS_EXTERNAL_ENGINE=1"
    } >> "$script_dir/.env"
else
    touch "/tmp/stub-secrets-engine.sock"
    echo "DOCKER_PASS_EXTERNAL_ENGINE=0" >> "$script_dir/.env"
    _export_docker_pass_secrets
fi
