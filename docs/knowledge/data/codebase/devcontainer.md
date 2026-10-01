---
type: codebase
description: The development container — compose setup around the ROS desktop image, lifecycle hooks that install tooling and the agentdev catalog, and firewall/MCP sidecars.
source: .devcontainer
source_digest: sha256:bdc55782931d610f1c13f11e7aa46c56a4a7e8ce407a26b993e6e1d42318e545
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
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
  - Features: `sshd` and `docker-in-docker` 4.1.1.
  - `initializeCommand` → `devcontainer-init.sh`.
  - `postCreateCommand`, `postStartCommand`, and `postAttachCommand` scripts;
    startup waits for `postStartCommand`.
- `docker-compose.yml`: the `devcontainer` and `mcp-gateway` services. The
  devcontainer runs `privileged: true` for GPU rendering and TTY servo access.
- `devcontainer-lock.json`: pinned feature versions.

## How it works

**`postCreateCommand`:**

1. Fixes the ownership of the mounted build, install, log, and cache volumes.
2. Links `~/.claude.json`.
3. Installs `codebase-memory-mcp`.
4. Runs `uv sync`.
5. When `AGENTDEV_CATALOG_DIR` is set, reinstalls the agentdev catalog for Codex
   and for Claude at user scope.

**`postStartCommand`:**

1. Starts codebase-memory.
2. Sets up git `safe.directory`, pre-commit, the keyring, and the firewall.
3. Starts Xpra, unless `AGENTDEV_SKIP_XPRA` is set.
4. Runs `scripts/workspace-extensions.sh` and VirtualHere.
5. Configures Codex.

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

Verified anchor points (line numbers as of 2026-09-26):

- `.devcontainer/devcontainer.json:11` — compose `service`
- `.devcontainer/devcontainer.json:141` — lifecycle hooks
- `.devcontainer/docker-compose.yml:57` — desktop image
- `.devcontainer/docker-compose.yml:64` — `privileged: true`
- `.devcontainer/scripts/postCreateCommand.sh` — tooling and catalog install
- `.devcontainer/scripts/postStartCommand.sh` — per-start services
