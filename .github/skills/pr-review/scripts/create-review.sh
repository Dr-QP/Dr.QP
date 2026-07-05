#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${script_dir}/__common.sh"

repo=""
pr_number=""

usage() {
  cat <<'EOF'
Create a pending GitHub PR review (no comments, no event yet).

Usage:
  create-review.sh --pr <PR_NUMBER> [--repo <owner/name>]

Prints on success:
  REVIEW_ID=<id>
  COMMIT_ID=<sha>

Exit codes:
  0  Success
  2  Bad usage / missing prerequisite
  1  gh command failed
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pr) require_arg "--pr" "${2:-}"; pr_number="$2"; shift 2 ;;
    --repo) require_arg "--repo" "${2:-}"; repo="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) print_error "Unknown argument: $1"; usage >&2; exit 2 ;;
  esac
done

require_arg "--pr" "${pr_number}"
require_gh

[[ -n "${repo}" ]] || repo="$(resolve_repo)"

commit_id="$(gh pr view "${pr_number}" --repo "${repo}" --json headRefOid -q .headRefOid)"

response="$(gh api "repos/${repo}/pulls/${pr_number}/reviews" -f commit_id="${commit_id}")"
review_id="$(printf '%s' "${response}" | jq -r '.id')"

printf 'REVIEW_ID=%s\n' "${review_id}"
printf 'COMMIT_ID=%s\n' "${commit_id}"
