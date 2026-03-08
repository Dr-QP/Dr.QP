#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${script_dir}/__common.sh"

remote_name="origin"
branch_name=""
remote_was_explicit=0

usage() {
  show_help_header "Ensure the current branch exists on the remote and push if needed."
  cat <<'EOF'

Usage:
  ensure-remote-branch.sh [--remote <name>] [--branch <name>]

Options:
  --remote <name>    Remote to verify and push to. Default: origin
  --branch <name>    Branch to verify. Default: current branch
  -h, --help         Show this help text.

Exit codes:
  0  Success or already up to date
  3  Branch is behind or diverged from its upstream
  4  Push failed
  5  Default branch warning

Examples:
  .github/skills/open-pr/scripts/ensure-remote-branch.sh
  .github/skills/open-pr/scripts/ensure-remote-branch.sh --remote origin --branch feature/my-change
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --remote)
      [[ $# -ge 2 ]] || { print_error "Missing value for --remote"; exit 2; }
      remote_name="$2"
      remote_was_explicit=1
      shift 2
      ;;
    --branch)
      [[ $# -ge 2 ]] || { print_error "Missing value for --branch"; exit 2; }
      branch_name="$2"
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

if [[ -z "${branch_name}" ]]; then
  branch_name="$(current_branch)"
fi

if ! git remote get-url "${remote_name}" >/dev/null 2>&1; then
  print_error "Remote not found: ${remote_name}"
  exit 2
fi

if ! git show-ref --verify --quiet "refs/heads/${branch_name}"; then
  print_error "Local branch not found: ${branch_name}"
  exit 2
fi

if is_default_branch "${branch_name}"; then
  printf 'WARNING: You are on %s. Create a feature branch before opening a PR unless the user explicitly confirms otherwise.\n' "${branch_name}" >&2
  exit 5
fi

upstream_ref=""
if upstream_ref="$(git rev-parse --abbrev-ref --symbolic-full-name "${branch_name}@{u}" 2>/dev/null)"; then
  upstream_remote="${upstream_ref%%/*}"
  upstream_branch="${upstream_ref#*/}"

  if [[ "${remote_was_explicit}" -eq 1 && "${upstream_remote}" != "${remote_name}" ]]; then
    print_error "Configured upstream remote '${upstream_remote}' does not match --remote '${remote_name}'."
    printf 'Please either rerun the command with --remote %s or update the branch upstream before continuing.\n' "${upstream_remote}" >&2
    exit 2
  fi

  ahead_count="$(git rev-list --count "${upstream_ref}..${branch_name}")"
  behind_count="$(git rev-list --count "${branch_name}..${upstream_ref}")"

  printf 'BRANCH=%s\n' "${branch_name}"
  printf 'UPSTREAM=%s\n' "${upstream_ref}"
  printf 'AHEAD=%s\n' "${ahead_count}"
  printf 'BEHIND=%s\n' "${behind_count}"

  if [[ "${behind_count}" -gt 0 && "${ahead_count}" -gt 0 ]]; then
    print_error "Cannot push: your branch has diverged from remote."
    printf 'Please resolve conflicts by merging the remote branch into your local branch:\n' >&2
    printf '  git pull %s %s\n' "${upstream_remote}" "${upstream_branch}" >&2
    printf 'Then resolve any merge conflicts, commit the merge, and run:\n' >&2
    printf '  git push\n' >&2
    exit 3
  elif [[ "${behind_count}" -gt 0 ]]; then
    print_error "Cannot push: your branch is behind its upstream remote."
    printf 'Please update your local branch from the configured upstream (a fast-forward pull is usually sufficient):\n' >&2
    printf '  git pull --ff-only %s %s\n' "${upstream_remote}" "${upstream_branch}" >&2
    printf 'Then run:\n' >&2
    printf '  git push\n' >&2
    exit 3
  fi

  if [[ "${ahead_count}" -eq 0 ]]; then
    printf 'ACTION=none\n'
    printf 'RESULT=up-to-date\n'
    exit 0
  fi

  printf 'ACTION=push\n'
  if push_output="$(git push 2>&1)"; then
    printf '%s\n' "${push_output}"
    printf 'RESULT=pushed\n'
    exit 0
  fi

  print_error "Git push failed. Please check your Git credentials."
  printf '%s\n' "${push_output}" >&2
  exit 4
fi

printf 'BRANCH=%s\n' "${branch_name}"
printf 'UPSTREAM=%s/%s\n' "${remote_name}" "${branch_name}"
printf 'ACTION=push-with-upstream\n'

if push_output="$(git push -u "${remote_name}" "${branch_name}" 2>&1)"; then
  printf '%s\n' "${push_output}"
  printf 'RESULT=pushed-with-upstream\n'
  exit 0
fi

print_error "Git push failed. Please check your Git credentials."
printf '%s\n' "${push_output}" >&2
exit 4
