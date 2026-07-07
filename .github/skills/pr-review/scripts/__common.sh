#!/usr/bin/env bash

set -euo pipefail

print_error() {
  printf 'ERROR: %s\n' "$*" >&2
}

require_arg() {
  local name="$1" value="$2"
  if [[ -z "${value}" ]]; then
    print_error "Missing required argument: ${name}"
    exit 2
  fi
}

require_gh() {
  command -v gh >/dev/null 2>&1 || { print_error "gh CLI not found on PATH."; exit 2; }
  gh auth status >/dev/null 2>&1 || { print_error "gh is not authenticated."; exit 2; }
}

require_body_file() {
  local body_file="$1"
  if [[ ! -f "${body_file}" ]]; then
    print_error "Body file not found: ${body_file}. Write the text with the Write tool first, then pass its path."
    exit 2
  fi
}

# Prints "<owner>/<name>" for the current repo
resolve_repo() {
  gh repo view --json nameWithOwner -q .nameWithOwner
}
