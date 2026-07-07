#!/usr/bin/env bash
set -euo pipefail

# Bootstrap a Codex Cloud host so repository agents can use GitHub CLI.
#
# Codex Cloud tasks have no ROS environment, so this script does not set up
# or run ROS build/test/lint commands.

REPO_ROOT=$(git rev-parse --show-toplevel 2>/dev/null || pwd)
cd "$REPO_ROOT"

log() {
    printf '\n==> %s\n' "$*"
}

have() {
    command -v "$1" >/dev/null 2>&1
}

run_sudo() {
    if [[ ${EUID:-$(id -u)} -eq 0 ]]; then
        "$@"
    elif have sudo; then
        sudo "$@"
    else
        printf 'error: %s requires root privileges and sudo is unavailable\n' "$*" >&2
        return 1
    fi
}

install_github_cli() {
    if have gh; then
        log "GitHub CLI already installed: $(gh --version | head -n 1)"
        return 0
    fi

    log "Installing GitHub CLI"
    if have apt-get; then
        run_sudo mkdir -p /etc/apt/keyrings
        curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg \
            | run_sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg >/dev/null
        run_sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg
        printf 'deb [arch=%s signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main\n' \
            "$(dpkg --print-architecture)" \
            | run_sudo tee /etc/apt/sources.list.d/github-cli.list >/dev/null
        run_sudo apt-get update
        run_sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y gh
    else
        printf 'error: unsupported package manager; install GitHub CLI before running Codex tasks\n' >&2
        return 1
    fi
}

verify_github_auth() {
    if gh auth status >/dev/null 2>&1; then
        log "GitHub CLI is authenticated"
        return 0
    fi

    if [[ -n "${GH_TOKEN:-${GITHUB_TOKEN:-}}" ]]; then
        log "GitHub token detected in environment; gh can use it for non-interactive commands"
        return 0
    fi

    log "GitHub CLI is installed but not authenticated"
    cat <<'MSG'
Set GH_TOKEN or GITHUB_TOKEN in the Codex Cloud environment, or run `gh auth login`
interactively before tasks that need GitHub API access.
MSG
}

main() {
    install_github_cli
    verify_github_auth

    log "Codex Cloud environment is ready"
}

main "$@"
