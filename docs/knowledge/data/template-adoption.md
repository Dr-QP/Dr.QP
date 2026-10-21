---
type: tracker
stage: living
description: Which agent-devcontainer template ref this repository reflects, how it was adopted, and the choices that shape every template update.
generated:
  by: claude-code/opus-5.5
  at: 2026-10-21T01:40:00Z
sources:
- resource: .agent.metadata.json
- resource: .agentdev-template-progress.md
---

# Template adoption

This repository consumes the
[`plume-works/agent-devcontainer`](https://github.com/plume-works/agent-devcontainer)
template. The authoritative adopted ref is `template-consume.consumed_ref` in
the root `.agent.metadata.json`; it is `5392c1c` as of the 2026-10-21 update.
The per-path decisions and evidence for every episode live in
`.agentdev-template-progress.md`, which is the live record.

## Adoption

- **Workflow B**, a merge into an existing repository: Dr.QP is a ROS 2 colcon
  workspace with its own source, CI, devcontainer, and lint configuration.
- **Optional bundles:** `custom-image` and `knowledge-base`. The ROS images
  under `docker/ros/` are consumer source, so the template's `docker/`,
  `ansible/`, `ansible.cfg`, and `.dockerignore` are not tracked.
- **Project memory** is onboarded: [product](product.md) comes from the
  maintainer interview and [codebase](codebase.md) from `/agentdev:iwe-map`.
  `docs/knowledge/data/` is consumer memory and is never reseeded.

## Settled choices

- Ruff stays in `ruff.toml`; the template's `.ruff.toml` would silently shadow
  it, so it is not adopted or tracked.
- `workspaceFolder` stays `/opt/ros/overlay_ws`, the path the colcon build and
  install tree are built against.
- The development image is `jazzy-ros-desktop`, built `FROM` the digest-pinned
  `agent-desktop`. Renovate owns every image digest and tool pin.
- The `agentdev` catalog comes from the installed plugin. `.claude/` keeps only
  the ROS- and project-specific skills.
- OpenCode is enabled in the devcontainer. The standalone `agent-opencode`
  runtime is not, because this repository does not build that image.
- Agents use the `iwe` CLI; there is no iwe MCP server.
- Image publication is gated: merge jobs push `sha-<commit>` candidate tags, and
  `promote` moves the moving tags after the tests pass. This repository's
  `promote` adds a `prefix` input for the act image's `act-<distro>-` tags.
- Each Coder agent start prunes Dev Container engine images older than 72 hours.
