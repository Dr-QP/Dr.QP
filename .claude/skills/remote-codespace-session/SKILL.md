---
name: remote-codespace-session
description: Create, safely sync, use, and stop a GitHub Codespace as this repository's remote ROS 2 build and test machine. Use when ROS is absent locally and Docker is unavailable, or when asked to run work through GitHub Codespaces.
---

# Remote Codespace Session

Use this workflow only when the local ROS wrapper cannot run and no Docker
daemon is available. With Docker, use
[microvm-sandbox](../microvm-sandbox/SKILL.md). The bundled scripts derive the
remote checkout as `/workspaces/<repository-name>` from `gh repo view`; do not
replace it with the local `/opt/ros/overlay_ws` path.

## Prerequisites

- `gh` is authenticated with `repo` and `codespace` scopes.
- `rsync` is installed locally when syncing uncommitted work.
- Run commands from this repository. Scripts store session data in `./.tmp/`.

## Workflow

1. Preview the first possible Codespace creation:

   ```bash
   .claude/skills/remote-codespace-session/scripts/codespace-ensure.sh --dry-run
   ```

   If it reports `ACTION=create`, show the user the proposed machine,
   timeouts, retention, and branch, then obtain explicit approval before
   creating the billable resource. `ACTION=reuse` needs no approval.

2. Create or reuse the approved Codespace:

   ```bash
   .claude/skills/remote-codespace-session/scripts/codespace-ensure.sh
   ```

3. Sync local work:

   ```bash
   .claude/skills/remote-codespace-session/scripts/codespace-sync.sh
   ```

   A clean local tree is pushed and synced with git. A dirty local tree is
   copied with rsync without `--delete`, then the script removes only
   Git-tracked paths deleted locally; remote-only files remain intact. In
   either case it refuses a dirty Codespace checkout or one with commits that
   are absent from `origin`. Commit/push or recover those remote commits—or
   explicitly return the remote checkout to an `origin` commit—before retrying.
   Keep local work as the source of truth once the remote checkout is safe.

4. Run the original colcon command. The exec script applies
   `scripts/with-ros-env.sh` itself, so do not include that wrapper again:

   ```bash
   .claude/skills/remote-codespace-session/scripts/codespace-exec.sh \
     python3 -m colcon build --symlink-install \
     --packages-up-to <package_name>
   ```

   Use command shapes from
   [ros2-workspace-build](../ros2-workspace-build/SKILL.md) and
   [ros2-workspace-testing](../ros2-workspace-testing/SKILL.md). Re-sync after
   each local edit.

5. Retrieve an artifact when needed. Generate an SSH config explicitly (do
   not rely on a prior dirty-tree sync), and derive the same workspace path the
   scripts use:

   ```bash
   name="$(<./.tmp/codespace-name)"
   gh codespace ssh -c "$name" --config > ./.tmp/codespace-ssh-config
   host="$(awk '/^Host /{print $2; exit}' ./.tmp/codespace-ssh-config)"
   remote_workspace_dir="/workspaces/$(gh repo view --json name -q .name)"
   rsync -az -e "ssh -F ./.tmp/codespace-ssh-config" \
     "${host}:${remote_workspace_dir}/log/latest_test/" ./log/latest_test/
   ```

6. Stop the Codespace when the session ends. Delete it only with explicit user
   direction:

   ```bash
   .claude/skills/remote-codespace-session/scripts/codespace-teardown.sh
   .claude/skills/remote-codespace-session/scripts/codespace-teardown.sh --delete
   ```

## Bundled scripts

- [codespace-ensure.sh](scripts/codespace-ensure.sh) creates or reuses a
  Codespace. Its help describes machine and polling options.
- [codespace-sync.sh](scripts/codespace-sync.sh) uses the non-destructive sync
  policy above.
- [codespace-exec.sh](scripts/codespace-exec.sh) runs one wrapped ROS command
  remotely and returns that command's exit code.
- [codespace-teardown.sh](scripts/codespace-teardown.sh) stops or deletes the
  recorded Codespace.
