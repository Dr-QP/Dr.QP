#!/usr/bin/env bash
set -euo pipefail

seed_dir="${AGENTDEV_AUTH_SEED_DIR:-/run/agentdev-auth-seed}"

seed_credential() {
    local source="$1"
    local target="$2"
    local label="$3"
    local lock_fd
    local temp

    [[ -s "$source" ]] || {
        if [[ -s "$target" ]]; then
            chmod 600 "$target"
        fi
        rm -f "$source"
        return 0
    }

    mkdir -p "$(dirname "$target")"
    chmod 700 "$(dirname "$target")"
    exec {lock_fd}>"${target}.seed.lock"
    chmod 600 "${target}.seed.lock"
    flock "$lock_fd"

    if [[ -s "$target" ]]; then
        chmod 600 "$target"
        rm -f "$source"
        return 0
    fi

    temp="$(mktemp "${target}.seed.XXXXXX")"
    chmod 600 "$temp"
    if ! python3 -c '
import json
import sys

with open(sys.argv[1]) as source:
    value = json.load(source)
if not isinstance(value, dict):
    raise SystemExit("credential must be a JSON object")
json.dump(value, sys.stdout, separators=(",", ":"))
' "$source" >"$temp"; then
        rm -f "$temp" "$source"
        echo "Invalid $label credential seed; credentials were not installed." >&2
        return 1
    fi

    mv -f "$temp" "$target"
    chmod 600 "$target"
    rm -f "$source"
    echo "Seeded $label credentials."
}

seed_credential \
    "$seed_dir/claude.json" \
    "${AGENTDEV_CLAUDE_AUTH_PATH:-/root/.agents-auth/claude/.credentials.json}" \
    Claude
seed_credential \
    "$seed_dir/codex.json" \
    "${AGENTDEV_CODEX_AUTH_PATH:-/root/.agents-auth/codex/auth.json}" \
    Codex
