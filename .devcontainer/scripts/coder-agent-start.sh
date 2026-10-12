#!/usr/bin/env bash
# The Coder agent's entry point, run by the template's coder_script on every
# agent start; see spec/devcontainer-git-credentials.

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! "$script_dir/setup-gh-credential-helper.sh"; then
  echo "Warning: git credential helper setup failed; starting Remote Control anyway." >&2
fi

exec "$script_dir/claude-remote-control-start.sh"
