# 01 — Rebuild the dev image on `agent-desktop`

Rebase `docker/ros/desktop/ros-desktop.dockerfile` onto
`ghcr.io/chocobot-farm/agent-desktop`, reducing this workspace's Ansible tree to
the ROS delta.

**Depends on:** nothing in this repo. Requires
[`chocobot-farm/agent-devcontainer`](https://github.com/chocobot-farm/agent-devcontainer)
publishing `agent-desktop:edge` multi-arch.

**Blocked by:** finding F7 in [`README.md`](README.md) — decide the rebuild
trigger before starting.

## Acceptance criteria

- **AC1** `docker/ros/desktop/ros-desktop.dockerfile` starts from a digest-pinned
  `ghcr.io/chocobot-farm/agent-desktop` and runs a ROS-only playbook.
- **AC2** The built image is functionally identical to today's for every
  workspace command: `colcon build`, `colcon test`, `ament_flake8`,
  `rosdep install`, Gazebo launch tests, Xpra, DinD.
- **AC3** `/opt/ros/overlay_ws` still resolves correctly inside the devcontainer
  without changing `.devcontainer/devcontainer.json`'s `workspaceFolder`.
- **AC4** The 18 roles now supplied by the base image are deleted from
  `docker/ros/ansible/roles/`, not merely unreferenced.
- **AC5** A stale base image cannot go unnoticed: either Renovate updates the
  digest pin, or upstream dispatches a rebuild.
- **AC6** CI passes on native amd64 **and** arm64.

## Test plan (write first)

1. **Image content parity.** Extend the existing image smoke coverage with a test
   asserting every tool this workspace depends on is present and on `PATH`:
   `colcon`, `rosdep`, `vcs`, `ament_flake8`, `clang`/`clang++` at the pinned
   version, `ros2`, plus the generic set (`uv`, `bun`, `gh`, `cmake`, `zizmor`,
   `xpra`). Run it against the newly built image before touching CI.
2. **Workspace-path regression.** Assert that with `DEV_WORKSPACE_FOLDER=/opt/ros/overlay_ws`
   the `gh` shim resolves and the firewall allowlist default points into
   `/opt/ros/overlay_ws/.devcontainer/`. This is the F1 guarantee; it must fail
   if the base image ever hardcodes its path again.
3. **`ros_distro` guard.** Assert the ROS playbook still fails fast when
   `ros_distro` is undefined (F5), since the base image's `extra_facts` no longer
   does.
4. **`ros_dev_tools` completeness.** Diff the apt package list in the new
   `ros_dev_tools` role against today's `dev_tools` list minus the generic
   packages; assert the sets match exactly. Cheap, and catches a dropped package
   before a build does.

## Implementation

### Step 1 — new base and playbook

`docker/ros/desktop/ros-desktop.dockerfile`:

```dockerfile
ARG FROM_IMAGE=ghcr.io/chocobot-farm/agent-desktop:edge
FROM $FROM_IMAGE

ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=$ROS_DISTRO
ARG CLANG_VERSION=20
ARG OVERLAY_WS=/opt/ros/overlay_ws

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,readonly,source=..,target=/ros-scripts \
    apt-get update \
    && cd /ros-scripts/ansible \
    && ansible-playbook playbooks/20_ros_setup.yml \
      -i inventories/localhost.yml -vvv \
      -e "clang_version=$CLANG_VERSION ros_distro=$ROS_DISTRO"
```

The four `install_*=true` extra-vars go away — those capabilities now come from
the base image. Keep the `WORKDIR`, the LLVM `PATH` prepend, `CC`/`CXX`, and
`ros_entrypoint.sh` (the base image's `/entrypoint.sh` is a bare `exec "$@"` and
does not source ROS).

`20_ros_setup.yml` becomes:

```yaml
- name: Setup ROS 2 Environment
  hosts: all
  become: true
  gather_facts: true
  pre_tasks:
    - name: Ensure ros_distro is set # F5: base image's extra_facts no longer asserts
      ansible.builtin.fail:
        msg: 'ros_distro is not set'
      when: ros_distro is undefined
  roles:
    - { role: extra_facts, tags: ['extra_facts', 'always'] }
    - { role: ros_repo, tags: ['ros_repo'] }
    - { role: osrf_repo, tags: ['osrf_repo'] }
    - { role: ros_dev_tools, tags: ['ros_dev_tools'] }
    - { role: ros_shell_setup, tags: ['ros_shell_setup'] }
    - { role: cmake_system, tags: ['cmake'], when: source_install }
    - { role: colcon_setup, tags: ['colcon_setup'] }
    - { role: rosdep, tags: ['rosdep'] }
    - {
        role: ros_install_prebuilt,
        tags: ['ros_install'],
        when: not source_install,
      }
    - { role: ros_install_source, tags: ['ros_install'], when: source_install }
    - { role: ros_patches, tags: ['ros_patches'] }
    - { role: ros_dependencies, tags: ['ros_dependencies'] }
    - { role: clang, tags: ['clang'] }
    - {
        role: virtualhere_usb_client,
        tags: ['virtualhere_usb_client'],
        when: install_virtualhere_usb_client | default(false) | bool,
      }
```

`extra_facts` is retained locally because the base image's copy runs at build
time, not in this playbook's run.

### Step 2 — variable alias (F2)

In `docker/ros/ansible/playbooks/group_vars/all.yml`:

```yaml
# The base image renamed ros_user -> dev_user. Alias rather than editing the
# retained ROS roles.
ros_user: '{{ dev_user }}'
```

Keep `ros_distro`, `source_install`, `clang_version`, and
`install_virtualhere_usb_client`. Drop `install_xpra`, `install_docker`,
`install_agentic_tools`, `setup_user`, and the `ros_user_setup_*` block.

### Step 3 — the two new roles (F3)

Create `roles/ros_dev_tools/` and `roles/ros_shell_setup/` per F3 in
[`README.md`](README.md). Lift the package list and the shell templates
verbatim from git history (`git show HEAD~1:docker/ros/ansible/roles/dev_tools/tasks/main.yml`)
so nothing is retyped.

### Step 4 — delete the superseded roles (AC4)

```text
basic_prereqs  locale_setup  utc_timezone  bash_setup  fish_setup
cmake_kitware  dev_tools     github_cli    bun_setup   nodejs
uv_setup       install_docker install_docker_service   agentic_tools
devcontainer_firewall  xpra_setup  ros_user_setup
```

`dev_tools/defaults/main.yml` holds the pinned `zizmor` version that
`scripts/validate-super-linter-tool-versions.sh` reads — retarget that script at
the upstream value or keep the defaults file alone.

`docker/ros/desktop/start-xpra.sh` and `docker/ros/bin/gh` are also now supplied
by the base image; delete the local copies and update
`scripts/devcontainer-postStartCommand.sh` to call `/start-xpra.sh`.

### Step 5 — CI (AC5, AC6)

In `.github/workflows/ci.yml`, delete the `build-dev-image` job's `base_refs`
warm-start step and the `ubuntu-ansible` build. Pin the base:

```yaml
FROM_IMAGE=ghcr.io/chocobot-farm/agent-desktop@sha256:<digest>
```

Then add the rebuild trigger — Renovate on the digest, or accept a
`repository_dispatch` from upstream. Pick one; AC5 is not satisfied by "we'll
remember".

## Risks

- **Silent tool drift.** An upstream bump of `bun`/`uv`/`cmake` lands in this
  workspace without a Dr.QP-side review. Test 1 is the guard.
- **Package omission in `ros_dev_tools`.** A missing `python3-colcon-*` surfaces
  as a confusing colcon failure, not a build error. Test 4 is the guard.
- **Xpra/VirtualGL divergence.** Gazebo and RViz depend on the base image's
  VirtualGL build. If upstream drops or bumps it, GPU rendering breaks in a way
  unit tests will not catch — verify a real Gazebo launch test after each bump.
