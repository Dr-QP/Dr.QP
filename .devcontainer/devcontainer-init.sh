#!/usr/bin/env bash
set -xeuo pipefail

script_dir=$(dirname "${BASH_SOURCE[0]}")

gitdir=$(realpath "$(git rev-parse --git-common-dir)")
echo "GIT_REPO=$gitdir" > "$script_dir/.env"

LOCAL_WORKSPACE_FOLDER=$(realpath "$script_dir/..")
echo "LOCAL_WORKSPACE_FOLDER=$LOCAL_WORKSPACE_FOLDER" >> "$script_dir/.env"

# Export docker/mcp/ secrets from the host via `docker pass` so the container can import them.
# The export file is written to .tmp/ (gitignored) and is read+deleted by devcontainer-setup-pass.sh.
_export_docker_pass_secrets() {
    local export_dir="$LOCAL_WORKSPACE_FOLDER/.tmp"
    local export_file="$export_dir/docker-pass-export"

    # Requires the docker-pass CLI plugin (docker/secrets-engine) to be installed on the host.
    if ! docker pass ls >/dev/null 2>&1; then
        return 0
    fi

    # `docker pass ls` prints one full secret path per line, e.g.:
    #   docker/mcp/github.personal_access_token
    mkdir -p "$export_dir"
    : > "$export_file"
    chmod 600 "$export_file"

    # `docker pass get` always masks the value (hardcoded "**********").
    # The only way to extract plaintext is to let Docker inject it via the se:// URI scheme.
    local name value
    while IFS= read -r name; do
        value=$(docker run --rm -e "_V=se://$name" busybox sh -c 'printf "%s" "$_V"' 2>/dev/null) || continue
        printf '%s\t%s\n' "$name" "$value" >> "$export_file"
    done < <(docker pass ls 2>/dev/null | grep '^docker/mcp/')
}
_export_docker_pass_secrets
