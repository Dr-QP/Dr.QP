#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${script_dir}/__common.sh"

delete=0

usage() {
  show_help_header "Stop (default) or delete the codespace recorded by codespace-ensure.sh."
  cat <<'EOF'

Usage:
  codespace-teardown.sh [--delete]

Options:
  --delete            Fully delete the codespace instead of stopping it.
                       Skips gh's own interactive confirmation via --force.
  -h, --help          Show this help text.

Behavior:
  Default: 'gh codespace stop' -- stops compute billing, keeps storage so a
  later codespace-ensure.sh can reuse it. Does not touch ./.tmp/codespace-name
  or ./.tmp/codespace-ssh-config.
  --delete: 'gh codespace delete --force' -- fully deletes the codespace, then
  removes ./.tmp/codespace-name and ./.tmp/codespace-ssh-config since they now
  refer to a codespace that no longer exists.

Exit codes:
  0  Stopped or deleted successfully
  2  Usage error, gh missing, plain auth failure, or no codespace name recorded
  3  Under-scoped token, or the gh codespace stop/delete command itself failed
     (check the printed ERROR message to tell which one occurred)

Examples:
  .github/skills/remote-codespace-session/scripts/codespace-teardown.sh
  .github/skills/remote-codespace-session/scripts/codespace-teardown.sh --delete
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --delete)
      delete=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      print_error "Unknown argument: $1"
      usage >&2
      exit 2
      ;;
  esac
done

require_git_repo
require_gh
require_codespace_auth

name="$(read_codespace_name)"

if [[ "${delete}" -eq 1 ]]; then
  status=0
  delete_output="$(gh codespace delete -c "${name}" --force 2>&1)" || status=$?
  if [[ "${status}" -ne 0 ]]; then
    print_error "gh codespace delete failed: ${delete_output}"
    exit 3
  fi

  rm -f "$(codespace_name_file)" "$(ssh_config_file)"

  printf 'ACTION=delete\n'
  printf 'CODESPACE=%s\n' "${name}"
  exit 0
fi

status=0
stop_output="$(gh codespace stop -c "${name}" 2>&1)" || status=$?
if [[ "${status}" -ne 0 ]]; then
  print_error "gh codespace stop failed: ${stop_output}"
  exit 3
fi

printf 'ACTION=stop\n'
printf 'CODESPACE=%s\n' "${name}"
exit 0
