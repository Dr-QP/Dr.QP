---
type: codebase
description: The development container — compose setup around the ROS desktop image, lifecycle hooks that install tooling and the agentdev catalog, and firewall/MCP sidecars.
source: .devcontainer
source_digest: sha256:1af16142716e96a0f4fd779e207e649730d1e82cbf02442eeca8af7d1330dd8d
verified:
  by: claude-code/opus-5.5
  at: 2026-10-12T22:32:35Z
stale_after: 2027-01-10
generated:
  by: claude-code/opus-5.5
  at: 2026-10-12T22:32:35Z
sources:
- id: code
  resource: .devcontainer
---

# devcontainer

The supported development environment. It is a Docker Compose devcontainer: the
`devcontainer` service runs `ghcr.io/dr-qp/jazzy-ros-desktop:edge` with the
workspace at `/opt/ros/overlay_ws`, next to a `mcp-gateway` sidecar
(`docker/mcp-gateway`). It is consumed from the `plume-works/agent-devcontainer`
template (`.agent.metadata.json`).

## Public surface

- `devcontainer.json`:
  - Features: `sshd` and `docker-in-docker` 4.1.3.
  - `initializeCommand` → `devcontainer-init.sh`.
  - `postCreateCommand`, `postStartCommand`, and `postAttachCommand` scripts;
    startup waits for `postStartCommand`.
- `docker-compose.yml`: the `devcontainer` and `mcp-gateway` services. The
  devcontainer runs `privileged: true` for GPU rendering and TTY servo access.
  It mounts the checkout at `/opt/ros/overlay_ws` and again at
  `/workspaces/<basename>`, the path the Coder agent and template tooling
  expect; `devcontainer-init.sh` writes `WORKSPACE_BASENAME` to `.env`.
- `scripts/coder-agent-start.sh`: the Coder agent's start hook. It sets up the
  gh credential helper, then `exec`s `claude-remote-control-start.sh`.
- `devcontainer-lock.json`: pinned feature versions.

## How it works

**`postCreateCommand`:**

1. Fixes the ownership of the mounted build, install, log, and cache volumes.
2. Links `~/.claude.json`.
3. Installs `codebase-memory-mcp`; a failure only warns.
4. Pre-approves the Claude workspace and links Codex's `auth.json`.
5. Runs `uv sync`.
6. When `AGENTDEV_CATALOG_DIR` is set, reinstalls the agentdev catalog for
   Codex, for Claude at user scope, and for OpenCode.

**`postStartCommand`:**

1. Starts codebase-memory.
2. Sets up git `safe.directory`, pre-commit, the keyring, the gh credential
   helper, and the firewall.
3. Starts Xpra, unless `AGENTDEV_SKIP_XPRA` is set.
4. Runs `scripts/workspace-extensions.sh` and VirtualHere.
5. Configures Codex and re-links its `auth.json`.

## Depends on

- [docker/ros](docker/ros.md) (the desktop image)

## Invariants & gotchas

- The agentdev catalog install rewrites `.claude/settings.json`. The resulting
  diff is expected output of the hooks.
- The firewall is opt-in. `firewall.sh` does nothing unless
  `ENABLE_FIREWALL=true`, and then runs the image's `init-firewall.sh`.
  `devcontainer.json` points to `firewall-allowlist.txt` for the allowed
  destinations.

## Key references

Verified anchor points (line numbers as of 2026-10-12):

- `.devcontainer/devcontainer.json:11` — compose `service`
- `.devcontainer/devcontainer.json:141` — lifecycle hooks
- `.devcontainer/docker-compose.yml:57` — desktop image
- `.devcontainer/docker-compose.yml:64` — `privileged: true`
- `.devcontainer/docker-compose.yml:86` — `/workspaces/<basename>` mount
- `.devcontainer/devcontainer-init.sh:34` — `WORKSPACE_BASENAME`
- `.devcontainer/scripts/coder-agent-start.sh` — Coder agent start hook
- `.devcontainer/scripts/postCreateCommand.sh` — tooling and catalog install
- `.devcontainer/scripts/postStartCommand.sh` — per-start services
