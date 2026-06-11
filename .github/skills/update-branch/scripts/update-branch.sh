#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${script_dir}/__common.sh"

remote_name="origin"
base_branch="main"

usage() {
  show_help_header "Merge origin/main into the current branch."
  cat <<'EOF'

Usage:
  update-branch.sh [--remote <name>] [--base <branch>]

Options:
  --remote <name>    Remote to fetch from. Default: origin
  --base <branch>    Base branch to merge. Default: main
  -h, --help         Show this help text.

Exit codes:
  0  Merge succeeded (fast-forward or clean merge)
  1  Merge conflicts detected — resolve manually and commit
  2  Usage error or not a git repo
  3  Already up to date
  5  Current branch is the default branch — refusing to self-merge

Examples:
  .github/skills/update-branch/scripts/update-branch.sh
  .github/skills/update-branch/scripts/update-branch.sh --remote upstream --base main
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --remote)
      [[ $# -ge 2 ]] || { print_error "Missing value for --remote"; exit 2; }
      remote_name="$2"
      shift 2
      ;;
    --base)
      [[ $# -ge 2 ]] || { print_error "Missing value for --base"; exit 2; }
      base_branch="$2"
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

branch_name="$(current_branch)"

if is_default_branch "${branch_name}"; then
  print_error "Current branch is '${branch_name}'. Refusing to merge '${base_branch}' into the default branch."
  exit 5
fi

# Preflight: clean working tree
if [[ -n "$(git status --porcelain)" ]]; then
  print_error "Working tree is dirty. Commit or stash changes before updating the branch."
  git status --short >&2
  exit 2
fi

printf 'BRANCH=%s\n' "${branch_name}"
printf 'REMOTE=%s\n' "${remote_name}"
printf 'BASE=%s/%s\n' "${remote_name}" "${base_branch}"

# Fetch
printf 'Fetching %s...\n' "${remote_name}"
git fetch "${remote_name}"

base_ref="${remote_name}/${base_branch}"

# Check if already up to date
merge_base="$(git merge-base HEAD "${base_ref}")"
base_sha="$(git rev-parse "${base_ref}")"

if [[ "${merge_base}" == "${base_sha}" ]]; then
  printf 'RESULT=up-to-date\n'
  printf 'Already up to date with %s.\n' "${base_ref}"
  exit 3
fi

# Attempt merge
printf 'Merging %s into %s...\n' "${base_ref}" "${branch_name}"
if git merge --no-ff "${base_ref}" --message "Merge ${base_ref} into ${branch_name}"; then
  printf 'RESULT=merged\n'
  git log --oneline --decorate -n 5
  exit 0
fi

# Conflicts detected
printf 'RESULT=conflicts\n'
print_error "Merge conflicts detected. Resolve the conflicts listed below, then run:"
printf '  git add <resolved-files>\n' >&2
printf '  git commit\n' >&2
printf '\nConflicted files:\n' >&2
git diff --name-only --diff-filter=U >&2
exit 1
