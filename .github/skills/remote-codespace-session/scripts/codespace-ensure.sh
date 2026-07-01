#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${script_dir}/__common.sh"

branch=""
machine="standardLinux32gb"
idle_timeout="30m"
retention_period="1d"
poll_interval=10
poll_timeout=300
dry_run=0

usage() {
  show_help_header "Find-or-create the Codespace used as this session's remote build/test machine."
  cat <<'EOF'

Usage:
  codespace-ensure.sh [--branch <name>] [--machine <name>] [--idle-timeout <dur>]
                       [--retention-period <dur>] [--poll-interval <seconds>]
                       [--poll-timeout <seconds>] [--dry-run]

Options:
  --branch <name>             Branch to create/find the codespace for. Default: current branch
  --machine <name>            gh codespace create -m value. Default: standardLinux32gb
  --idle-timeout <dur>        gh codespace create --idle-timeout value. Default: 30m
  --retention-period <dur>    gh codespace create --retention-period value. Default: 1d
  --poll-interval <seconds>   Seconds between state polls while waiting for a new codespace. Default: 10
  --poll-timeout <seconds>    Total seconds to wait before giving up. Default: 300
  --dry-run                   Print what would happen without creating anything or writing
                               ./.tmp/codespace-name
  -h, --help                  Show this help text.

Exit codes:
  0  Resolved (reused or created); or dry-run report printed
  2  Usage error, gh missing, or plain auth failure
  3  Under-scoped token (missing 'codespace' OAuth scope)
  4  gh codespace create failed
  5  Timed out waiting for the new codespace to reach Available

Examples:
  .github/skills/remote-codespace-session/scripts/codespace-ensure.sh
  .github/skills/remote-codespace-session/scripts/codespace-ensure.sh --dry-run
  .github/skills/remote-codespace-session/scripts/codespace-ensure.sh --branch feature/my-change --machine premiumLinux
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --branch)
      [[ $# -ge 2 ]] || { print_error "Missing value for --branch"; exit 2; }
      branch="$2"
      shift 2
      ;;
    --machine)
      [[ $# -ge 2 ]] || { print_error "Missing value for --machine"; exit 2; }
      machine="$2"
      shift 2
      ;;
    --idle-timeout)
      [[ $# -ge 2 ]] || { print_error "Missing value for --idle-timeout"; exit 2; }
      idle_timeout="$2"
      shift 2
      ;;
    --retention-period)
      [[ $# -ge 2 ]] || { print_error "Missing value for --retention-period"; exit 2; }
      retention_period="$2"
      shift 2
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
    --dry-run)
      dry_run=1
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

owner_repo="$(resolve_owner_repo)"

if [[ -z "${branch}" ]]; then
  branch="$(current_branch)"
fi

display_name="codex-$(branch_slug "${branch}")"

existing_name="$(codespace_lookup_by_display_name "${owner_repo}" "${display_name}")"

if [[ -n "${existing_name}" ]]; then
  state="$(codespace_state_by_name "${owner_repo}" "${existing_name}")"

  if [[ "${state}" != "Available" && "${state}" != "Shutdown" ]]; then
    printf 'NOTE: Codespace %s has unexpected state %s; reusing it anyway. A later ssh call is the real usability gate.\n' "${existing_name}" "${state}" >&2
  fi

  if [[ "${dry_run}" -eq 1 ]]; then
    printf 'ACTION=reuse\n'
    printf 'CODESPACE=%s\n' "${existing_name}"
    printf 'STATE=%s\n' "${state}"
    exit 0
  fi

  write_codespace_name "${existing_name}"
  printf 'ACTION=reuse\n'
  printf 'CODESPACE=%s\n' "${existing_name}"
  printf 'STATE=%s\n' "${state}"
  exit 0
fi

if [[ "${dry_run}" -eq 1 ]]; then
  printf 'ACTION=create\n'
  printf 'DISPLAY_NAME=%s\n' "${display_name}"
  printf 'MACHINE=%s\n' "${machine}"
  printf 'IDLE_TIMEOUT=%s\n' "${idle_timeout}"
  printf 'RETENTION_PERIOD=%s\n' "${retention_period}"
  printf 'BRANCH=%s\n' "${branch}"
  exit 0
fi

create_status=0
create_stderr_file="$(repo_root_tmp_dir)/codespace-ensure-create-stderr.$$"
create_stdout="$(gh codespace create -R "${owner_repo}" -b "${branch}" \
  --devcontainer-path .devcontainer/devcontainer.json \
  -m "${machine}" --idle-timeout "${idle_timeout}" \
  --retention-period "${retention_period}" --default-permissions \
  --display-name "${display_name}" 2>"${create_stderr_file}")" || create_status=$?
create_stderr="$(cat "${create_stderr_file}" 2>/dev/null || true)"
rm -f "${create_stderr_file}"

new_name="$(printf '%s' "${create_stdout}" | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//')"

if [[ "${create_status}" -ne 0 || -z "${new_name}" ]]; then
  print_error "gh codespace create failed: ${create_stderr}"
  exit 4
fi

elapsed=0
state="$(codespace_state_by_name "${owner_repo}" "${new_name}")"
while [[ "${state}" != "Available" ]]; do
  if [[ "${elapsed}" -ge "${poll_timeout}" ]]; then
    print_error "Timed out after ${poll_timeout}s waiting for codespace ${new_name} to become Available (last state: ${state}). Check 'gh codespace list' manually."
    exit 5
  fi
  sleep "${poll_interval}"
  elapsed=$((elapsed + poll_interval))
  state="$(codespace_state_by_name "${owner_repo}" "${new_name}")"
done

write_codespace_name "${new_name}"
printf 'ACTION=create\n'
printf 'CODESPACE=%s\n' "${new_name}"
printf 'STATE=Available\n'
exit 0
