#!/usr/bin/env bash

# Shared helpers for the remote-codespace-session skill scripts.
# This file is meant to be sourced, not executed directly:
#   source "$(dirname -- "${BASH_SOURCE[0]}")/__common.sh"

set -euo pipefail

skill_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
skill_root_dir="$(cd -- "${skill_script_dir}/.." && pwd)"

print_error() {
  printf 'ERROR: %s\n' "$*" >&2
}

require_git_repo() {
  if ! git rev-parse --show-toplevel >/dev/null 2>&1; then
    print_error "This script must be run inside a Git repository."
    exit 2
  fi
}

require_gh() {
  if ! command -v gh >/dev/null 2>&1; then
    print_error "The GitHub CLI ('gh') is required but was not found on PATH."
    exit 2
  fi
}

require_codespace_auth() {
  local err
  if ! err="$(gh codespace list --json name -q 'length' 2>&1 >/dev/null)"; then
    if printf '%s' "${err}" | grep -qiE '403|scope'; then
      print_error "gh is authenticated but the token is missing the 'codespace' OAuth scope. This skill needs a token with 'repo' + 'codespace' scopes (set GH_TOKEN/GITHUB_TOKEN, or run 'gh auth login' with those scopes)."
      exit 3
    fi
    print_error "gh is not authenticated. Run 'gh auth status' to check, then 'gh auth login'."
    exit 2
  fi
}

current_branch() {
  git rev-parse --abbrev-ref HEAD
}

resolve_owner_repo() {
  gh repo view --json owner,name -q '.owner.login + "/" + .name'
}

repo_name() {
  gh repo view --json name -q '.name'
}

branch_slug() {
  local branch_name="$1"
  printf '%s' "${branch_name}" \
    | tr '[:upper:]' '[:lower:]' \
    | sed -e 's/[^a-z0-9-]\{1,\}/-/g' -e 's/^-\{1,\}//' -e 's/-\{1,\}$//' \
    | cut -c1-40
}

codespace_display_name() {
  local branch_name="$1"
  printf 'codex-%s' "$(branch_slug "${branch_name}")"
}

# GitHub Codespaces clones the repository into /workspaces/<repo-name> and
# runs the devcontainer from there, regardless of the docker-compose bind
# mount this repo uses locally (/opt/ros/overlay_ws). Derive that remote path
# from the repo name rather than hardcoding it.
codespace_workspace_dir() {
  printf '/workspaces/%s' "$(repo_name)"
}

repo_root_tmp_dir() {
  local tmp_dir
  tmp_dir="$(git rev-parse --show-toplevel)/.tmp"
  mkdir -p "${tmp_dir}"
  printf '%s' "${tmp_dir}"
}

codespace_name_file() {
  printf '%s/codespace-name' "$(repo_root_tmp_dir)"
}

ssh_config_file() {
  printf '%s/codespace-ssh-config' "$(repo_root_tmp_dir)"
}

write_codespace_name() {
  local name="$1"
  printf '%s' "${name}" | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//' > "$(codespace_name_file)"
}

read_codespace_name() {
  local name_file
  name_file="$(codespace_name_file)"
  if [[ ! -s "${name_file}" ]]; then
    print_error "No codespace name recorded. Run codespace-ensure.sh first."
    exit 2
  fi
  sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//' < "${name_file}"
}

codespace_lookup_by_display_name() {
  local owner_repo="$1"
  local display_name="$2"
  gh codespace list -R "${owner_repo}" --json name,displayName -q ".[] | select(.displayName==\"${display_name}\") | .name" | head -n1
}

codespace_state_by_name() {
  local owner_repo="$1"
  local name="$2"
  gh codespace list -R "${owner_repo}" --json name,state -q ".[] | select(.name==\"${name}\") | .state" | head -n1
}

show_help_header() {
  local description="$1"

  printf '%s\n\n' "${description}"
  printf 'Skill root: %s\n' "${skill_root_dir}"
}
