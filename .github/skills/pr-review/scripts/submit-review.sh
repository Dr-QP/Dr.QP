#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${script_dir}/__common.sh"

repo=""
pr_number=""
review_id=""
event=""
body_file=""

usage() {
  cat <<'EOF'
Submit a pending PR review, publishing all of its inline comments.

Usage:
  submit-review.sh --pr <PR_NUMBER> --review-id <id> \
    --event <COMMENT|REQUEST_CHANGES|APPROVE> --body-file <path> \
    [--repo <owner/name>]

The summary body must be in a plain file (write it with the Write tool
first). It must be a short overall summary only -- no per-finding
detail, since findings already live in the inline comments.

Exit codes:
  0  Success
  2  Bad usage / missing prerequisite
  1  gh command failed
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pr) require_arg "--pr" "${2:-}"; pr_number="$2"; shift 2 ;;
    --review-id) require_arg "--review-id" "${2:-}"; review_id="$2"; shift 2 ;;
    --event) require_arg "--event" "${2:-}"; event="$2"; shift 2 ;;
    --body-file) require_arg "--body-file" "${2:-}"; body_file="$2"; shift 2 ;;
    --repo) require_arg "--repo" "${2:-}"; repo="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) print_error "Unknown argument: $1"; usage >&2; exit 2 ;;
  esac
done

require_arg "--pr" "${pr_number}"
require_arg "--review-id" "${review_id}"
require_arg "--event" "${event}"
require_arg "--body-file" "${body_file}"
require_body_file "${body_file}"

case "${event}" in
  COMMENT|REQUEST_CHANGES|APPROVE) ;;
  *) print_error "Invalid --event: ${event} (expected COMMENT, REQUEST_CHANGES, or APPROVE)"; exit 2 ;;
esac

require_gh

[[ -n "${repo}" ]] || repo="$(resolve_repo)"

payload="$(jq -n \
  --arg event "${event}" \
  --rawfile body "${body_file}" \
  '{event: $event, body: $body}')"

printf '%s' "${payload}" | gh api "repos/${repo}/pulls/${pr_number}/reviews/${review_id}/events" --input -
