#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${script_dir}/__common.sh"

repo=""
pr_number=""
commit_id=""
file_path=""
line=""
side="RIGHT"
body_file=""

usage() {
  cat <<'EOF'
Attach one inline comment to the caller's pending review for a PR.

GitHub has no "attach to review by id" field on the review-comments
endpoint -- a comment created here while the authenticated actor has an
open PENDING review on this PR attaches to that review automatically.
Passing pull_request_review_id instead returns HTTP 422 (no matching
oneOf schema); do not add it.

Usage:
  add-inline-comment.sh --pr <PR_NUMBER> --commit <sha> --path <file> \
    --line <n> [--side RIGHT|LEFT] --body-file <path> [--repo <owner/name>]

The comment text must live in a plain file (write it with the Write
tool first) so arbitrary finding text never needs shell-escaping.

Exit codes:
  0  Success
  2  Bad usage / missing prerequisite
  1  gh command failed
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pr) require_arg "--pr" "${2:-}"; pr_number="$2"; shift 2 ;;
    --commit) require_arg "--commit" "${2:-}"; commit_id="$2"; shift 2 ;;
    --path) require_arg "--path" "${2:-}"; file_path="$2"; shift 2 ;;
    --line) require_arg "--line" "${2:-}"; line="$2"; shift 2 ;;
    --side) require_arg "--side" "${2:-}"; side="$2"; shift 2 ;;
    --body-file) require_arg "--body-file" "${2:-}"; body_file="$2"; shift 2 ;;
    --repo) require_arg "--repo" "${2:-}"; repo="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) print_error "Unknown argument: $1"; usage >&2; exit 2 ;;
  esac
done

require_arg "--pr" "${pr_number}"
require_arg "--commit" "${commit_id}"
require_arg "--path" "${file_path}"
require_arg "--line" "${line}"
require_arg "--body-file" "${body_file}"
require_body_file "${body_file}"
require_gh

[[ -n "${repo}" ]] || repo="$(resolve_repo)"

payload="$(jq -n \
  --arg commit_id "${commit_id}" \
  --arg path "${file_path}" \
  --argjson line "${line}" \
  --arg side "${side}" \
  --rawfile body "${body_file}" \
  '{commit_id: $commit_id, path: $path, line: $line, side: $side, body: $body}')"

printf '%s' "${payload}" | gh api "repos/${repo}/pulls/${pr_number}/comments" --input -
