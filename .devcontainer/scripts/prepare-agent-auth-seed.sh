#!/usr/bin/env bash
set -euo pipefail

seed_dir="${AGENTDEV_AUTH_SEED_DIR:?AGENTDEV_AUTH_SEED_DIR is required}"
mkdir -p "$seed_dir"
chmod 700 "$seed_dir"

prepare_credential() {
    local env_name="$1"
    local filename="$2"
    local label="$3"
    local target="$seed_dir/$filename"
    local temp
    local value="${!env_name:-}"

    rm -f "$target"
    [[ -n "$value" ]] || return 0

    temp="$(mktemp "$seed_dir/.${filename}.XXXXXX")"
    chmod 600 "$temp"
    if ! printf '%s' "$value" | python3 -c '
import json
import sys

value = json.load(sys.stdin)
if not isinstance(value, dict):
    raise SystemExit("credential must be a JSON object")
json.dump(value, sys.stdout, separators=(",", ":"))
' >"$temp"; then
        rm -f "$temp"
        echo "Invalid JSON in $env_name; $label seed was not prepared." >&2
        return 1
    fi

    mv -f "$temp" "$target"
    chmod 600 "$target"
    echo "Prepared $label credential seed."
}

prepare_credential AGENTDEV_CLAUDE_JSON claude.json Claude
prepare_credential AGENTDEV_CODEX_JSON codex.json Codex
