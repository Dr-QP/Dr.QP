#!/usr/bin/env bash
set -euo pipefail

# Bootstrap a Codex Cloud host so repository agents can use GitHub CLI and the
# VS Code Dev Containers CLI before entering the project devcontainer.
#
# This script is intentionally host-focused. ROS build/test/lint commands still
# belong inside the devcontainer via `devcontainer exec` and
# `scripts/with-ros-env.sh`.

REPO_ROOT=$(git rev-parse --show-toplevel 2>/dev/null || pwd)
cd "$REPO_ROOT"

TMP_DIR="./.tmp/codex-cloud-setup"
mkdir -p "$TMP_DIR"

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

install_devcontainer_cli() {
    if have devcontainer; then
        log "Dev Containers CLI already installed: $(devcontainer --version)"
        return 0
    fi

    log "Installing Dev Containers CLI"
    if have npm; then
        run_sudo npm install -g @devcontainers/cli
    elif have bun; then
        run_sudo bun add -g @devcontainers/cli
    else
        printf 'error: npm or bun is required to install @devcontainers/cli\n' >&2
        return 1
    fi
}

ensure_docker_running() {
    if ! have docker; then
        printf 'error: Docker CLI is not installed on the Codex Cloud host\n' >&2
        return 1
    fi

    if docker info >/dev/null 2>&1; then
        log "Docker daemon is already running"
        return 0
    fi

    log "Starting Docker daemon"
    if have service; then
        run_sudo service docker start || true
    fi
    if ! docker info >/dev/null 2>&1 && have systemctl; then
        run_sudo systemctl start docker || true
    fi
    if ! docker info >/dev/null 2>&1; then
        printf 'error: Docker daemon is not reachable; start Docker before running devcontainer commands\n' >&2
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

bring_up_devcontainer() {
    log "Bringing up the devcontainer"
    devcontainer up --workspace-folder /workspace \
        --mount "type=bind,source=/var/run/docker.sock,target=/var/run/docker.sock"
}

main() {
    install_github_cli
    install_devcontainer_cli
    ensure_docker_running
    verify_github_auth
    bring_up_devcontainer

    log "Codex Cloud environment is ready"
    cat <<'MSG'
Run commands inside the devcontainer with:
  devcontainer exec --workspace-folder /workspace bash -lc '<command>'

Run ROS commands through the repository wrapper, for example:
  devcontainer exec --workspace-folder /workspace bash -lc 'scripts/with-ros-env.sh ros2 --help'
MSG
}

main "$@"
