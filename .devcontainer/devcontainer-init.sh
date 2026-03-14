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

    # Build a sh one-liner that prints "name\tvalue" for each secret.
    local print_cmd="" j
    for j in $(seq 0 $((i - 1))); do
        print_cmd+="printf '%s\t%s\n' \"\$_N${j}\" \"\$_S${j}\"; "
    done

    mkdir -p "$export_dir"
    : > "$export_file"
    chmod 600 "$export_file"

    docker run --rm "${args[@]}" busybox sh -c "$print_cmd" 2>/dev/null >> "$export_file"
}
_export_docker_pass_secrets
