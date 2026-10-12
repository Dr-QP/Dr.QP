#!/usr/bin/env bash
# HTTPS git reuses gh's login; see spec/devcontainer-git-credentials.

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Written by setup-keyring.sh; lets gh reach a login stored in the keyring.
keyring_env="$script_dir/../../.tmp/keyring-session.env"
if [[ -f "$keyring_env" ]]; then
  # shellcheck disable=SC1090
  source "$keyring_env"
fi

if ! command -v gh >/dev/null 2>&1; then
  echo "gh is not installed; skipping git credential helper setup."
  exit 0
fi

if ! gh auth status --hostname github.com >/dev/null 2>&1; then
  echo "gh is not authenticated to github.com; skipping git credential helper setup."
  exit 0
fi

if existing="$(git config --get-urlmatch credential.helper https://github.com)" &&
  [[ -n "$existing" ]]; then
  echo "A git credential helper for github.com is already configured; leaving it unchanged."
  exit 0
fi

gh auth setup-git
echo "Configured gh as the git credential helper."
