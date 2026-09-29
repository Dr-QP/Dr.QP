#!/usr/bin/env bash
# Renovate's one post-upgrade task: regenerate what a bump invalidates, then format it,
# so the branch passes CI without a foreign commit that would stop Renovate updating it.

set -euo pipefail

# renovate: datasource=npm depName=@devcontainers/cli
DEVCONTAINER_CLI_VERSION="0.89.0"

cd "$(git rev-parse --show-toplevel)"

# Renovate's edits are uncommitted in its clone, so the diff against HEAD is the bump.
changed_files()
{
  git diff --name-only --diff-filter=d HEAD
  git ls-files --others --exclude-standard
}

mapfile -t changed < <(changed_files)
if [[ ${#changed[@]} -eq 0 ]]; then
  exit 0
fi

if printf '%s\n' "${changed[@]}" | grep -qx '.devcontainer/devcontainer.json'; then
  bunx --package "@devcontainers/cli@${DEVCONTAINER_CLI_VERSION}" \
    devcontainer upgrade --workspace-folder .
fi

# pre-commit through uv (the dev dependency group pins it), not whatever the image ships.
# Post-upgrade runs on the base branch, so the default-branch commit guard must not fire.
export SKIP="${SKIP:+${SKIP},}no-commit-to-branch"
mapfile -t changed < <(changed_files)
if ! uv run --frozen pre-commit run --files "${changed[@]}"; then
  mapfile -t changed < <(changed_files)
  uv run --frozen pre-commit run --files "${changed[@]}"
fi
