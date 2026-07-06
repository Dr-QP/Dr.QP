#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${script_dir}/__common.sh"

repo=""
pr_number=""
event=""
summary_file=""
comments_file=""

usage() {
  cat <<'EOF'
Create and submit a GitHub PR review with all inline comments in a
single atomic call -- no pending-review/add-comment/submit dance.

Usage:
  post-review.sh --pr <PR_NUMBER> --event <COMMENT|APPROVE> \
    --summary-file <path> [--comments-file <path>] [--repo <owner/name>]

--summary-file must be a plain file (write it with the Write tool
first) containing only the short overall summary -- no per-finding
detail, since findings live in the inline comments.

--comments-file, if given, must be a plain JSON file (write it with
the Write tool first) containing an array of findings, e.g.:
  [
    {"path": "src/foo.py", "line": 42, "side": "RIGHT", "body": "..."},
    {"path": "src/bar.cpp", "line": 7, "side": "RIGHT", "body": "..."}
  ]
Omit --comments-file (or pass a file containing []) when there are no
validated findings to attach.

Exit codes:
  0  Success
  2  Bad usage / missing prerequisite
  1  gh command failed
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pr) require_arg "--pr" "${2:-}"; pr_number="$2"; shift 2 ;;
    --event) require_arg "--event" "${2:-}"; event="$2"; shift 2 ;;
    --summary-file) require_arg "--summary-file" "${2:-}"; summary_file="$2"; shift 2 ;;
    --comments-file) require_arg "--comments-file" "${2:-}"; comments_file="$2"; shift 2 ;;
    --repo) require_arg "--repo" "${2:-}"; repo="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) print_error "Unknown argument: $1"; usage >&2; exit 2 ;;
  esac
done

require_arg "--pr" "${pr_number}"
require_arg "--event" "${event}"
require_arg "--summary-file" "${summary_file}"
require_body_file "${summary_file}"

case "${event}" in
  COMMENT|APPROVE) ;;
  *) print_error "Invalid --event: ${event} (expected COMMENT or APPROVE)"; exit 2 ;;
esac

require_gh

if [[ -n "${comments_file}" ]]; then
  require_body_file "${comments_file}"
else
  mkdir -p ./.tmp
  comments_file="./.tmp/post-review-empty-comments.json"
  printf '[]' >"${comments_file}"
fi

[[ -n "${repo}" ]] || repo="$(resolve_repo)"

commit_id="$(gh pr view "${pr_number}" --repo "${repo}" --json headRefOid -q .headRefOid)"

payload="$(jq -n \
  --arg commit_id "${commit_id}" \
  --arg event "${event}" \
  --rawfile body "${summary_file}" \
  --slurpfile comments_arr "${comments_file}" \
  '{commit_id: $commit_id, event: $event, body: $body, comments: $comments_arr[0]}')"

printf '%s' "${payload}" | gh api "repos/${repo}/pulls/${pr_number}/reviews" --method POST --input -
