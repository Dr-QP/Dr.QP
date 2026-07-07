#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${script_dir}/__common.sh"

usage() {
  show_help_header "Sync the local working tree onto the codespace created by codespace-ensure.sh."
  cat <<'EOF'

Usage:
  codespace-sync.sh

Options:
  -h, --help         Show this help text.

Behavior:
  If the working tree is clean (no uncommitted or untracked changes), pushes
  the current branch and syncs the codespace's checkout via 'git fetch' +
  'git reset --hard' over SSH.
  If the working tree is dirty, rsyncs the working tree directly to the
  codespace over SSH, excluding .git and everything git considers ignored
  (all .gitignore files in the tree, honoring '!' negations).

Exit codes:
  0  Synced (either path)
  2  Usage error, missing codespace name, or SSH config generation/parsing failure
  3  Under-scoped token, or git push failed (diverged or other push error)
  4  rsync failed
  5  Remote git fetch/checkout/reset over SSH failed

Examples:
  .github/skills/remote-codespace-session/scripts/codespace-sync.sh
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
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

name="$(read_codespace_name)"

if [[ -n "$(git status --porcelain)" ]]; then
  # Rsync path: working tree has uncommitted and/or untracked changes.
  require_rsync
  ssh_cfg="$(ssh_config_file)"

  if ! gh codespace ssh -c "${name}" --config > "${ssh_cfg}"; then
    print_error "Failed to generate SSH config for codespace ${name}."
    exit 2
  fi

  host="$(awk '/^Host /{print $2; exit}' "${ssh_cfg}")"
  if [[ -z "${host}" ]]; then
    print_error "Could not parse SSH host alias from generated config."
    exit 2
  fi

  repo_root="$(git rev-parse --show-toplevel)"
  exclude_file="$(sync_exclude_file)"

  # Ask git for its fully-resolved ignore list rather than feeding rsync the
  # root .gitignore directly: rsync's --exclude-from can't see the repo's
  # nested .gitignore files, and it has no concept of gitignore's '!'
  # negation, so files re-included by a negation line (e.g. .vscode/mcp.json)
  # would otherwise be silently dropped from every sync.
  git -C "${repo_root}" ls-files --others --ignored --exclude-standard \
    | sed 's|^|/|' > "${exclude_file}"

  if ! rsync_output="$(rsync -az --delete -e "ssh -F ${ssh_cfg}" \
    --exclude-from="${exclude_file}" --exclude '.git' \
    "${repo_root}/" "${host}:$(codespace_workspace_dir)/" 2>&1)"; then
    print_error "rsync to codespace failed:"
    printf '%s\n' "${rsync_output}" >&2
    exit 4
  fi

  printf 'ACTION=rsync\n'
  printf 'HOST=%s\n' "${host}"
  printf 'CODESPACE=%s\n' "${name}"
  exit 0
fi

# Git-push path: working tree is clean.
branch="$(current_branch)"

if git rev-parse --abbrev-ref --symbolic-full-name "${branch}@{u}" >/dev/null 2>&1; then
  push_cmd=(git push)
else
  push_cmd=(git push -u origin "${branch}")
fi

if ! push_output="$("${push_cmd[@]}" 2>&1)"; then
  print_error "Git push failed. Fetch/merge or rebase to resolve, then re-run:"
  printf '%s\n' "${push_output}" >&2
  exit 3
fi
printf '%s\n' "${push_output}"

if ! ssh_output="$(gh codespace ssh -c "${name}" -- "cd $(codespace_workspace_dir) && git fetch origin && git checkout ${branch} && git reset --hard origin/${branch}" 2>&1)"; then
  print_error "Remote sync over SSH failed:"
  printf '%s\n' "${ssh_output}" >&2
  exit 5
fi
printf '%s\n' "${ssh_output}"

printf 'ACTION=git-push\n'
printf 'BRANCH=%s\n' "${branch}"
printf 'CODESPACE=%s\n' "${name}"
exit 0
