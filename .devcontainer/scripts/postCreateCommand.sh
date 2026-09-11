#!/usr/bin/env bash
set -exuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="${DEV_WORKSPACE_FOLDER:-$(cd "$script_dir/../.." && pwd)}"

chown_up() {
  local owner="$1"
  local loop_dir="$2"
  while [[ -n "$loop_dir" && "$loop_dir" != "/" ]]; do
    sudo chown "$owner" "$loop_dir"
    loop_dir="$(dirname "$loop_dir")"
  done
}

# Fix ownership of the workspace and its parent directories.
# This was originally needed for GitHub Actions where workspace is owned by 1001:1001
# which blows up the CBM internal checks of ownership of the cache directory and workspace
# Fix is placed next to other ownership fixes as it might be needed for other tools
#
# Ownership alone is what CBM's ancestry check rejects; the mode is not part of
# it. The run that first passed had every component still at 0755 and only the
# owner corrected.
#
# Do NOT add a chmod here. This loop walks up to /, and narrowing the workspace
# parent to 0700 was tried upstream: it strips traverse for every non-root user,
# so the postStartCommand died with `spawn docker EACCES`. Worse, the workspace
# is a bind mount of the runner's checkout, so the mode change propagates back
# to the host and the rest of the job fails to read its own files.
chown_up "root:root" "$workspace"

# Fix ownership of the home directory and its parent directories. This is needed for
# GitHub Actions where $HOME is /github/home which is owned by 1001:1001
chown_up "root:root" "$HOME"

# Named volumes are created root-owned by the daemon; make sure the container
# user owns the mount points it writes to.
#
# Only chown what is actually mounted. In a devcontainer every one of these is
# present, so this is a no-op there. A caller that mounts none of them — a
# GitHub Actions `container:` job — would otherwise die here under `set -e`
# before any agent setup ran.
for mount_point in \
    "$workspace/.ansible" \
    "$workspace/.vscode" \
    "$workspace/.cache" \
    "$workspace/build" \
    "$workspace/install" \
    "$workspace/lcov" \
    "$workspace/log" \
    "$workspace/docs/_build" \
    "$workspace/.micromamba" \
    /uv; do
    if [[ -e "$mount_point" ]]; then
        sudo chown -R root:root "$mount_point"
    else
        echo "$mount_point is not mounted; skipping its ownership fix."
    fi
done

# Symlink ~/.claude.json into the volume BEFORE cbm-install so it folds its MCP
# entry back in (only happens when the path is already a symlink). Discard the
# image's real file; seed {} if fresh.
#
# ~/.claude.json is a single file, not a directory, and Docker named volumes are
# always directory-backed (mounting one straight at a file path materializes an
# empty directory there instead), so it is persisted as a plain file inside the
# already-mounted agentdev-claude volume.
if [[ -f /root/.claude.json && ! -L /root/.claude.json ]]; then
    rm -f /root/.claude.json
fi
if [[ ! -e /root/.claude/claude.json ]]; then
    echo '{}' >/root/.claude/claude.json
fi
ln -sf /root/.claude/claude.json /root/.claude.json

# Wire the codebase-memory-mcp binary staged by the image into this user's agent
# config now that the real ~/.claude and ~/.codex volumes are mounted. No-op on
# an image that does not carry the binary.
"$script_dir/codebase-memory-mcp-install.sh"

# Both agents' credential setup below needs their subdirectory of the shared
# agentdev-agents-auth volume to exist first.
mkdir -p /root/.agents-auth/claude /root/.agents-auth/codex
chmod 700 /root/.agents-auth/claude /root/.agents-auth/codex

# See link-codex-auth.sh for why Codex's auth.json needs the same file-in-a-shared-
# volume-plus-symlink treatment, and why this also has to run again from
# postStartCommand.sh.
"$script_dir/link-codex-auth.sh"

# Sync the project environment on the /uv volume so that extension settings are
# valid when the container is rebuilt. This is a no-op if the environment is
# already up to date.
"$script_dir/uv-sync.sh"

# Install the catalog staged in the image. This has to happen here rather than
# during the image build: the persistent ~/.claude and ~/.codex volumes mount over
# where both agents record installed plugins, so a build-time install would be
# shadowed for every container whose volume already exists. At user scope for
# Claude, so it applies to every workspace opened in this container.
if [[ -n "${AGENTDEV_CATALOG_DIR:-}" && -d "$AGENTDEV_CATALOG_DIR" ]]; then
    "$script_dir/reinstall-agentdev-codex.sh" "$AGENTDEV_CATALOG_DIR"
    "$script_dir/reinstall-agentdev-claude.sh" "$AGENTDEV_CATALOG_DIR" user
else
    echo "No catalog staged in the image; skipping the image-scoped plugin install."
fi
