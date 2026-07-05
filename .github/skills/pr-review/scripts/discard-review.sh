#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${script_dir}/__common.sh"

repo=""
pr_number=""
review_id=""

usage() {
  cat <<'EOF'
Delete a pending PR review instead of submitting it (e.g. every
candidate finding failed validation after the review was already
created).

Usage:
  discard-review.sh --pr <PR_NUMBER> --review-id <id> [--repo <owner/name>]

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
    --repo) require_arg "--repo" "${2:-}"; repo="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) print_error "Unknown argument: $1"; usage >&2; exit 2 ;;
  esac
done

require_arg "--pr" "${pr_number}"
require_arg "--review-id" "${review_id}"
require_gh

[[ -n "${repo}" ]] || repo="$(resolve_repo)"

gh api -X DELETE "repos/${repo}/pulls/${pr_number}/reviews/${review_id}"
