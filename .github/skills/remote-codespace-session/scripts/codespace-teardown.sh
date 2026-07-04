#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${script_dir}/__common.sh"

delete=0
poll_interval=10
poll_timeout=300

usage() {
  show_help_header "Stop (default) or delete the codespace recorded by codespace-ensure.sh."
  cat <<'EOF'

Usage:
  codespace-teardown.sh [--delete] [--poll-interval <seconds>] [--poll-timeout <seconds>]

Options:
  --delete                    Fully delete the codespace instead of stopping it.
                               Skips gh's own interactive confirmation via --force.
  --poll-interval <seconds>   Seconds between state polls while waiting for the
                               terminal state. Default: 10
  --poll-timeout <seconds>    Total seconds to wait for the terminal state before
                               giving up. Default: 300
  -h, --help                  Show this help text.

Behavior:
  Default: 'gh codespace stop' -- stops compute billing, keeps storage so a
  later codespace-ensure.sh can reuse it. Does not touch ./.tmp/codespace-name
  or ./.tmp/codespace-ssh-config. Waits (polling like codespace-ensure.sh) until
  the codespace reports 'Shutdown', since 'gh codespace stop' returns while the
  codespace is still 'ShuttingDown'.
  --delete: 'gh codespace delete --force' -- fully deletes the codespace, waits
  until it disappears from 'gh codespace list', then removes
  ./.tmp/codespace-name and ./.tmp/codespace-ssh-config since they now refer to
  a codespace that no longer exists.

Exit codes:
  0  Stopped (reached Shutdown) or deleted (gone from the list) successfully
  2  Usage error, gh missing, plain auth failure, or no codespace name recorded
  3  Under-scoped token, or the gh codespace stop/delete command itself failed
     (check the printed ERROR message to tell which one occurred)
  5  Timed out waiting for the codespace to reach the terminal state (Shutdown
     for stop, gone for delete). The stop/delete command itself was accepted;
     only the wait timed out. Re-check with 'gh codespace list'.

Examples:
  .github/skills/remote-codespace-session/scripts/codespace-teardown.sh
  .github/skills/remote-codespace-session/scripts/codespace-teardown.sh --delete
  .github/skills/remote-codespace-session/scripts/codespace-teardown.sh --poll-timeout 600
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --delete)
      delete=1
      shift
      ;;
    --poll-interval)
      [[ $# -ge 2 ]] || { print_error "Missing value for --poll-interval"; exit 2; }
      poll_interval="$2"
      shift 2
      ;;
    --poll-timeout)
      [[ $# -ge 2 ]] || { print_error "Missing value for --poll-timeout"; exit 2; }
      poll_timeout="$2"
      shift 2
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

owner_repo="$(resolve_owner_repo)"
name="$(read_codespace_name)"

if [[ "${delete}" -eq 1 ]]; then
  status=0
  delete_output="$(gh codespace delete -c "${name}" --force 2>&1)" || status=$?
  if [[ "${status}" -ne 0 ]]; then
    print_error "gh codespace delete failed: ${delete_output}"
    exit 3
  fi

  # Wait until the codespace no longer appears in the list.
  elapsed=0
  state="$(codespace_state_by_name "${owner_repo}" "${name}")"
  while [[ -n "${state}" ]]; do
    if [[ "${elapsed}" -ge "${poll_timeout}" ]]; then
      print_error "Timed out after ${poll_timeout}s waiting for codespace ${name} to be deleted (last state: ${state}). The delete was accepted; check 'gh codespace list'."
      exit 5
    fi
    sleep "${poll_interval}"
    elapsed=$((elapsed + poll_interval))
    state="$(codespace_state_by_name "${owner_repo}" "${name}")"
  done

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

# 'gh codespace stop' returns while the codespace is still 'ShuttingDown';
# wait until it settles at 'Shutdown' so callers don't reinvent this loop.
elapsed=0
state="$(codespace_state_by_name "${owner_repo}" "${name}")"
while [[ "${state}" != "Shutdown" ]]; do
  if [[ "${elapsed}" -ge "${poll_timeout}" ]]; then
    print_error "Timed out after ${poll_timeout}s waiting for codespace ${name} to reach Shutdown (last state: ${state}). The stop was accepted; check 'gh codespace list'."
    exit 5
  fi
  sleep "${poll_interval}"
  elapsed=$((elapsed + poll_interval))
  state="$(codespace_state_by_name "${owner_repo}" "${name}")"
done

printf 'ACTION=stop\n'
printf 'CODESPACE=%s\n' "${name}"
printf 'STATE=Shutdown\n'
exit 0
