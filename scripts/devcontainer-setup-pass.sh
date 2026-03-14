#!/usr/bin/env bash
# Sets up GPG + pass store inside the devcontainer and imports any docker/mcp/ secrets
# that were exported to .tmp/docker-pass-export by devcontainer-init.sh on the host.
set -xeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "$script_dir/.." && pwd)"

GPG_EMAIL="docker-secrets@local"
GPG_NAME="Docker Secrets"
EXPORT_FILE="$workspace_dir/.tmp/docker-pass-export"

# Generate a passphrase-less GPG key for unattended use if none exists yet.
if ! gpg --list-keys "$GPG_EMAIL" >/dev/null 2>&1; then
    echo "Generating GPG key for $GPG_EMAIL..."
    gpg --batch --gen-key <<EOF
Key-Type: RSA
Key-Length: 3072
Subkey-Type: RSA
Subkey-Length: 3072
Name-Real: $GPG_NAME
Name-Email: $GPG_EMAIL
Expire-Date: 0
%no-protection
EOF
fi

GPG_ID=$(gpg --list-keys --with-colons "$GPG_EMAIL" | awk -F: '/^fpr/ { print $10; exit }')

# Initialise the pass store if it has not been set up yet.
if [[ ! -f ~/.password-store/.gpg-id ]]; then
    echo "Initialising pass store with key $GPG_ID..."
    pass init "$GPG_ID"
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
        printf '%s\n' "$value" | pass insert --multiline --force "$name"
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
