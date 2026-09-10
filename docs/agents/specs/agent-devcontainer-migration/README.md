# agent-devcontainer migration — spike findings

Investigation into whether Dr.QP's development image should be rebuilt on top of
[`chocobot-farm/agent-devcontainer`](https://github.com/chocobot-farm/agent-devcontainer),
the general-purpose agent devcontainer extracted from this workspace.

**Status:** spike complete, not scheduled. Implementation guidance in
[`01-base-image-migration.md`](01-base-image-migration.md).

## Verdict

| Question                                                | Answer                                                    |
| ------------------------------------------------------- | --------------------------------------------------------- |
| Rebuild the dev image on `agent-desktop` as a base?     | **Yes — viable, and worth doing.** ~1 PR on each side.    |
| Share the `.claude` skills/agents catalog across repos? | **No — not yet.** No mechanism exists that does not hurt. |

## Context

`chocobot-farm/agent-devcontainer` publishes two multi-arch images to GHCR:

- `ghcr.io/chocobot-farm/ubuntu-ansible` — Ubuntu 24.04 + Ansible 13.4.0
- `ghcr.io/chocobot-farm/agent-desktop` — the development image

`agent-desktop` already contains 18 of the roles this workspace's
`20_ros_setup.yml` runs: `extra_facts`, `basic_prereqs`, `locale_setup`,
`utc_timezone`, `bash_setup`, `fish_setup`, `cmake_kitware`, `dev_tools`,
`github_cli`, `bun_setup`, `nodejs`, `uv_setup`, `install_docker`,
`install_docker_service`, `agentic_tools`, `devcontainer_firewall`,
`xpra_setup`, and `dev_user_setup` (this workspace's `ros_user_setup`).

Verified against the published image: `bun` 1.3.14, `node` 24.18.1, `uv` 0.12.0,
`gh` 2.96.0, `cmake` 4.4.1, `zizmor` 1.22.0, `claude` 2.1.220, `codex` 0.146.0,
plus `xpra`, `vglrun`, `init-firewall.sh`, `gnome-keyring-daemon`, `docker`.

## Findings

### F1 — The workspace-path indirection is already in place (priority: none, resolved)

This was the one hard blocker, and the extraction resolved it. `agent-desktop`
bakes its default workspace path (`/workspaces/project`) into only two places:
the `github_cli` PATH shim and the firewall allowlist default. Both read
`DEV_WORKSPACE_FOLDER` from the environment first:

```bash
# from /root/.bashrc in the published image
default_ws=/workspaces/project
ws="${DEV_WORKSPACE_FOLDER:-${GITHUB_WORKSPACE:-$default_ws}}"
```

`.devcontainer/devcontainer.json` in this workspace already sets
`DEV_WORKSPACE_FOLDER`, so `/opt/ros/overlay_ws` keeps working with no change.

One caveat: the shim looks for `$ws/docker/bin`, whereas this workspace keeps its
`gh` wrapper at `docker/ros/bin`. Either move the wrapper or accept the baked-in
`/usr/local/bin/gh` copy, which is identical.

### F2 — Variable rename reaches only 6 lines (priority: low)

`agent-desktop` renamed `ros_user` → `dev_user` and `ros_user_setup_*` →
`dev_user_setup_*`. In the ROS roles this workspace would retain, that touches:

| File                                      | Lines      | Reference               |
| ----------------------------------------- | ---------- | ----------------------- |
| `roles/colcon_setup/tasks/main.yml`       | 8,13,18,26 | `become_user: ros_user` |
| `roles/rosdep/tasks/main.yml`             | 24         | `become_user: ros_user` |
| `roles/rosdep/tasks/main.yml`             | 20         | `user_home`             |
| `roles/ros_install_source/tasks/main.yml` | 5          | `user_home`             |
| `roles/clang/tasks/main.yml`              | 29,39      | `user_home`             |

`user_home` keeps its name upstream, so only the `ros_user` references matter.
A single `ros_user: "{{ dev_user }}"` alias in `group_vars/all.yml` avoids
editing the roles at all.

### F3 — Two new roles absorb what the shared roles gave up (priority: medium)

`agent-desktop`'s `dev_tools` dropped the ROS packages, and its `bash_setup` /
`fish_setup` dropped the ROS shell integration. This workspace needs to re-add
them locally:

- **`ros_dev_tools`** — `python3-colcon-*` (cmake, common-extensions,
  coveragepy-result, lcov-result, mixin, output, package-information),
  `ros-dev-tools`, `python3-rosdep`, `python3-vcstool`, `lcov`, `libasio-dev`,
  `libbullet-dev`, `libcunit1-dev`, `libtinyxml2-dev`, `python3-scipy`,
  `python3-gz-transport13`, and the `python3-flake8-*` / `python3-pytest-*` sets.
- **`ros_shell_setup`** — the two `ros2_activate` / `ros2_ws` aliases from
  `bash_setup`, and the `ros.fish.j2` template (`ros2`/`colcon`/`rosidl`/
  `ament_index` completions, the `.micromamba/envs/ros-*` fallback).

Note `agent-desktop` keeps `ffmpeg` and `btop`, so those need no duplication.

### F4 — `ament_flake8` remains this workspace's gate (priority: low)

`agent-desktop` ships ruff only; its `python-lint-check.sh` is ruff-based. This
workspace's `ament_flake8` gate is unaffected because it keeps its own
`scripts/python-lint-check.sh`. No action, but do not assume the two scripts
stay in sync.

### F5 — `extra_facts` no longer asserts `ros_distro` (priority: medium)

Upstream deleted the `fail` task that made every playbook including
`extra_facts` ROS-coupled. This workspace must re-add the assert as a
`pre_tasks` entry in its own playbook, or lose the guard.

### F6 — The cyclic warm-start trick is lost (priority: medium, accepted cost)

`ci.yml` currently rebuilds `ubuntu-ansible` **on top of the previous
`jazzy-ros-desktop:edge`**, so apt and provisioning hit their caches. With an
external base, that becomes a digest pin on `agent-desktop`. The expensive layer
is the ROS install, which buildx still caches, so the practical cost is modest —
but the first build after each upstream bump is cold.

### F7 — Cross-repo CI coupling is the real ongoing cost (priority: high)

Nothing rebuilds this workspace's image when `agent-desktop:edge` moves. Needs a
Renovate rule on the digest pin, or a `repository_dispatch` from the upstream
repo's `merge-dev-image` job. Without it the base silently goes stale.

This is the main argument against migrating, and it should be decided before the
work is scheduled — not after.

### F8 — A shared `.claude` catalog has no good mechanism today (priority: high, blocks catalog sharing)

Git has no cross-repo symlink. The options:

| Option                                 | Why it fails today                                                                                                                         |
| -------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| Submodule at `.claude/skills/_shared/` | Breaks the flat skill-discovery layout and `validate_agent_files`' uniqueness assumptions; nested skill dirs are not discovered as skills. |
| Claude Code plugin marketplace entry   | Cleanest, but a different distribution mechanism; `create-skill` and `validate_agent_files` would need teaching about plugin paths.        |
| Vendor-and-sync script                 | Duplication with drift, plus a new thing to maintain. No better than the status quo.                                                       |

**Recommendation:** accept the duplication until the general catalog stabilizes,
then revisit via the plugin route. The 19 ROS-specific skills in this workspace
(`ros2-*`, `launch-testing`, `create-ros2-package-*`, …) never belonged upstream
anyway, so the shared surface is only the ~21 general skills.

## Costs summary

| Cost                                                    | Severity |
| ------------------------------------------------------- | -------- |
| Cross-repo CI coupling / staleness (F7)                 | High     |
| Two PRs + one image publish for a shared-tooling change | Medium   |
| Loss of the cyclic warm-start (F6)                      | Medium   |
| Two new local roles to write and maintain (F3)          | Medium   |
| Variable alias (F2), `ros_distro` assert (F5)           | Low      |

## Benefits summary

- One place to change shared dev tooling instead of two diverging copies.
- Dr.QP's image shrinks to the ROS delta; the generic layer is pulled prebuilt.
- Other `chocobot-farm` projects get the same environment for free — the reason
  the extraction happened.
- Forces the generic/ROS boundary to stay explicit rather than drifting back
  together.

## Next step

Nothing is scheduled. When it is, follow
[`01-base-image-migration.md`](01-base-image-migration.md), and resolve F7
first — a migration without a rebuild trigger is worse than no migration.
