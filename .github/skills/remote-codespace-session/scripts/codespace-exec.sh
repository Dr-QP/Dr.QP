#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${script_dir}/__common.sh"

usage() {
  show_help_header "Run a command on the codespace (created by codespace-ensure.sh, synced by codespace-sync.sh) via SSH."
  cat <<'EOF'

Usage:
  codespace-exec.sh <command> [args...]

Runs 'scripts/with-ros-env.sh <command> [args...]' on the codespace, inside
the repo's workspace directory, over 'gh codespace ssh'. Everything after
the script name is forwarded verbatim as the remote command -- including
'-h'/'--help', which is passed through to the remote command rather than
being intercepted by this script.

Example:
  codespace-exec.sh colcon build --packages-up-to drqp_brain --symlink-install

Exit codes:
  *  Whatever the remote command exits with, passed straight through.
     This is not "0 on success" in the usual sense -- 0 only if the
     remote command itself succeeded.
  2  No command given, gh missing, or plain auth failure / missing
     codespace name.
  3  Under-scoped token (missing 'codespace' OAuth scope).
EOF
}

require_git_repo
require_gh
require_codespace_auth

if [[ $# -eq 0 ]]; then
  usage >&2
  exit 2
fi

name="$(read_codespace_name)"

remote_cmd="cd $(codespace_workspace_dir) && scripts/with-ros-env.sh"
for arg in "$@"; do
  remote_cmd="${remote_cmd} $(printf '%q' "${arg}")"
done

status=0
gh codespace ssh -c "${name}" -- bash -lc "${remote_cmd}" || status=$?
exit "${status}"
