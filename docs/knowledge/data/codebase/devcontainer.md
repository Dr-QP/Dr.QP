---
type: codebase
description: The development container — compose setup around the ROS desktop image, lifecycle hooks that install tooling and the agentdev catalog, and firewall/MCP sidecars.
source: .devcontainer
source_digest: sha256:daa6b0574b3a763242c52bc321e42eeb2c1f015a33e79d160e3c69eea36ddf7c
verified:
  by: claude-code/opus-5.5
  at: 2026-10-21T01:35:27Z
stale_after: 2027-01-19
generated:
  by: claude-code/opus-5.5
  at: 2026-10-21T01:35:27Z
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
- Per-instance volumes for `~/.claude`, `~/.codex`, and OpenCode's data
  directory (`agentdev-opencode` at `/root/.local/share/opencode`).
- `scripts/coder-agent-start.sh`: the Coder agent's start hook. Each step only
  warns on failure: it prunes Dev Container engine images older than 72 hours,
  sets up the gh credential helper, and starts the OpenCode web UI (skipped when
  `opencode` or `tmux` is missing). Then it `exec`s
  `claude-remote-control-start.sh`.
- `scripts/setup-agent-state.sh`: the agent-state steps both lifecycle hooks
  call — `create`, `links`, `agent-policy`, and `catalog <dir> [scope]`.
- `devcontainer-lock.json`: pinned feature versions.

## How it works

**`postCreateCommand`:**

1. Fixes the ownership of the mounted build, install, log, and cache volumes.
2. `setup-agent-state.sh create`: links `~/.claude.json`, installs
   `codebase-memory-mcp` (a failure only warns), creates the shared auth
   directories, pre-approves the Claude workspace, and links Codex's
   `auth.json`.
3. Runs `uv sync`.
4. When `AGENTDEV_CATALOG_DIR` is set, `setup-agent-state.sh catalog` reinstalls
   the agentdev catalog for Codex, for Claude at user scope, and for OpenCode.

**`postStartCommand`:**

1. Starts codebase-memory.
2. Sets up git `safe.directory`, pre-commit, the keyring, the gh credential
   helper, and the firewall.
3. Starts Xpra, unless `AGENTDEV_SKIP_XPRA` is set.
4. Runs `scripts/workspace-extensions.sh` and VirtualHere.
5. `setup-agent-state.sh links` re-links Codex's `auth.json`; last,
   `agent-policy` writes the Codex and OpenCode permission policies.

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

Verified anchor points (line numbers as of 2026-10-21):

- `.devcontainer/devcontainer.json:11` — compose `service`
- `.devcontainer/devcontainer.json:141` — `agentdev-opencode` volume
- `.devcontainer/devcontainer.json:149` — lifecycle hooks
- `.devcontainer/docker-compose.yml:57` — desktop image
- `.devcontainer/docker-compose.yml:64` — `privileged: true`
- `.devcontainer/docker-compose.yml:86` — `/workspaces/<basename>` mount
- `.devcontainer/devcontainer-init.sh:34` — `WORKSPACE_BASENAME`
- `.devcontainer/scripts/coder-agent-start.sh:9` — Coder agent start hook
- `.devcontainer/scripts/docker-image-prune.sh:17` — 72-hour image prune
- `.devcontainer/scripts/setup-agent-state.sh:87` — subcommand dispatch
- `.devcontainer/scripts/postCreateCommand.sh:64` — agent state and catalog
- `.devcontainer/scripts/postStartCommand.sh:32` — per-start agent links
