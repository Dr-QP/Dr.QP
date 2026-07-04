#!/usr/bin/env bash
set -euo pipefail

script_dir=$(dirname "${BASH_SOURCE[0]}")

# GitHub Codespaces runs initializeCommand via `/bin/sh -c` with no HOME in the
# environment. The host MCP/secrets integration below is a local Docker Desktop
# convenience keyed off $HOME; under `set -u` an unbound HOME aborts the whole
# init (and thus the container build). Default it to empty so those host-path
# `-d`/`-S` probes simply fall through to their stub branches in Codespaces.
: "${HOME:=}"

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
    {
        echo "HOST_MCP_DIR=$HOST_MCP_DIR"
        echo "CONTAINER_MCP_DIR=$CONTAINER_MCP_DIR"
        echo "COMPOSE_PROFILES=mcp"
    } >> "$script_dir/.env"
    # Ensure the secrets socket stub exists so Docker Compose always bind-mounts a file,
    # not a directory (Linux creates a directory when the source path is missing).
    touch "/tmp/stub-secrets-engine.sock"
else
    # Ensure the container MCP dir exists to avoid bind mount errors, even if empty.
    mkdir -p "/tmp/stub-mcp"
fi

# docker/mcp/ secrets are served to the container by mounting the host's
# docker-secrets-engine socket directly (see HOST_SECRETS_SOCKET below); the
# gateway and the devcontainer service both talk to that socket, so no
# secret export/import step is needed here.
HOST_SECRETS_SOCKET="$HOME/Library/Caches/docker-secrets-engine/engine.sock"
DOCKER_SECRETS_ENGINE_SOCKET="/root/.cache/docker-secrets-engine/engine.sock"
if [[ -S "$HOST_SECRETS_SOCKET" ]]; then
    {
        echo "HOST_SECRETS_SOCKET=$HOST_SECRETS_SOCKET"
        echo "DOCKER_SECRETS_ENGINE_SOCKET=$DOCKER_SECRETS_ENGINE_SOCKET"
    } >> "$script_dir/.env"
else
    touch "/tmp/stub-secrets-engine.sock"
fi
