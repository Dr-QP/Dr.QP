#!/usr/bin/env bash
# Reproduce a GitHub Actions `container:` job locally and run the devcontainer
# lifecycle hooks inside it.
#
# A `container:` job provides the image and nothing else: none of
# devcontainer.json's `containerEnv`, none of its `mounts`. That difference is
# invisible from inside a devcontainer, where every variable and mount is always
# present, so hook changes that break CI look fine locally. This script makes
# that environment reproducible without pushing a commit -- a CI round trip is
# ~10 minutes, this is ~2.
#
# Usage:
#   .devcontainer/scripts/ci-hooks-repro.sh              # the contract the workflow supplies
#   .devcontainer/scripts/ci-hooks-repro.sh -e "FOO=bar" # add to it (ablation)
#   BARE=1 .devcontainer/scripts/ci-hooks-repro.sh       # supply nothing; expect failure
#
# Exit status is the hooks' own: 0 only when all three succeed.
set -euo pipefail

IMAGE="${IMAGE:-ghcr.io/plume-works/agent-desktop:edge}"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# Where GitHub mounts a checkout, and it runs the job as root.
workspace=/__w/agent-devcontainer/agent-devcontainer

# The minimal contract established by ablation; see the AI responder plan.
# BARE=1 drops it to show the failure it prevents.
contract=()
if [[ -z "${BARE:-}" ]]; then
    contract=(
        -e "CBM_CACHE_DIR=$workspace/.cache/codebase-memory-mcp"
        -e "UV_PROJECT_ENVIRONMENT=/opt/ci-venv"
    )
fi

# The checkout is copied in, never bind-mounted: with UV_PROJECT_ENVIRONMENT
# unset, uv-sync creates a .venv in the working tree, and a bind mount would
# leave that behind in the real repository.
docker run --rm \
    -v "$repo_root:/src:ro" \
    -e "DEV_WORKSPACE_FOLDER=$workspace" \
    -e AGENTDEV_SKIP_PRE_COMMIT=1 \
    -e AGENTDEV_SKIP_XPRA=1 \
    "${contract[@]}" \
    "$@" \
    "$IMAGE" \
    bash -c '
set -euo pipefail
workspace="$DEV_WORKSPACE_FOLDER"

# postCreateCommand.sh writes claude.json into ~/.claude, which a devcontainer
# supplies as a volume and a container: job does not.
mkdir -p "$workspace" /root/.claude /root/.codex
cp -a /src/. "$workspace/"
cd "$workspace"

# This repository is often developed in a git worktree, whose .git is a pointer
# file naming a host path the container cannot see. CI does an ordinary
# checkout, so re-init to keep that artifact from masking real findings.
if [[ ! -d .git ]]; then
    rm -f .git
    git init -q .
    git add -A >/dev/null 2>&1
    git -c user.email=ci@local -c user.name=ci commit -qm baseline >/dev/null 2>&1
fi

status=0
mkdir -p ./.tmp
for hook in postCreate postStart postAttach; do
    echo "=================== $hook ==================="
    if ! ".devcontainer/scripts/${hook}Command.sh" >"./.tmp/$hook.log" 2>&1; then
        echo "!!! $hook FAILED"
        tail -15 "./.tmp/$hook.log"
        status=1
        break
    fi
    echo "--- $hook OK"
done

echo "=================== side effects ==================="
if [[ -e .venv ]]; then
    echo "FAIL: uv wrote .venv into the checkout (UV_PROJECT_ENVIRONMENT unset)"
    status=1
else
    echo "ok: no .venv in the checkout"
fi

exit "$status"
'
