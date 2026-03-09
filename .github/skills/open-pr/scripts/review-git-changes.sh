#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${script_dir}/__common.sh"

base_ref="origin/main"
commit_range=""
include_patch=1

usage() {
  show_help_header "Review working tree and branch changes for the open-pr skill."
  cat <<'EOF'

Usage:
  review-git-changes.sh [--base-ref <ref>] [--range <range>] [--stat-only]

Options:
  --base-ref <ref>   Base ref used to build the default comparison range.
                     Default: origin/main
  --range <range>    Explicit git revision range, for example main..HEAD.
  --stat-only        Skip the full patch and print summary sections only.
  -h, --help         Show this help text.

Examples:
  .github/skills/open-pr/scripts/review-git-changes.sh
  .github/skills/open-pr/scripts/review-git-changes.sh --base-ref upstream/main
  .github/skills/open-pr/scripts/review-git-changes.sh --range origin/main..HEAD --stat-only
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --base-ref)
      [[ $# -ge 2 ]] || { print_error "Missing value for --base-ref"; exit 2; }
      base_ref="$2"
      shift 2
      ;;
    --range)
      [[ $# -ge 2 ]] || { print_error "Missing value for --range"; exit 2; }
      commit_range="$2"
      shift 2
      ;;
    --stat-only)
      include_patch=0
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

if [[ -z "${commit_range}" ]]; then
  if ! git rev-parse --verify --quiet "${base_ref}" >/dev/null; then
    print_error "Base ref not found: ${base_ref}"
    exit 2
  fi
  commit_range="${base_ref}..HEAD"
fi

if ! git rev-parse --verify --quiet HEAD >/dev/null; then
  print_error "Repository does not contain any commits yet."
  exit 2
fi

printf '== Branch ==\n'
printf '%s\n\n' "$(current_branch)"

printf '== Status ==\n'
git status --short --branch
printf '\n'

printf '== Diff Stat (%s) ==\n' "${commit_range}"
git --no-pager diff --stat "${commit_range}"
printf '\n'

if [[ "${include_patch}" -eq 1 ]]; then
  printf '== Working Tree Diff ==\n'
  git --no-pager diff
  printf '\n'

  printf '== Commit Range Diff (%s) ==\n' "${commit_range}"
  git --no-pager diff "${commit_range}"
  printf '\n'
fi

printf '== Commit Log (%s) ==\n' "${commit_range}"
git --no-pager log --oneline "${commit_range}"

if [[ -z "$(git status --porcelain)" ]] && [[ -z "$(git rev-list --max-count=1 "${commit_range}")" ]]; then
  printf '\nNOTE: No working tree changes and no commits detected in %s.\n' "${commit_range}"
fi
