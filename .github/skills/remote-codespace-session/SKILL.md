---
name: remote-codespace-session
description: "Spin up, sync, and drive a GitHub Codespace as a remote build/test machine running this repo's ROS 2 devcontainer. Use when asked to build/test in a sandbox without Docker support, when devcontainer exec is unavailable (e.g. Codex Tasks, no Docker daemon), or when asked to create/use/tear down a Codespace. Keywords: GitHub Codespaces, gh codespace, remote build, remote test, no docker, codex tasks, ssh build."
---

# Remote Codespace Session

Codex Tasks and similar sandboxes have no Docker daemon, so the
`devcontainer exec` workflow documented in
[microVM-sandbox.instructions.md](../../instructions/microVM-sandbox.instructions.md)
cannot run there. This repo's `.devcontainer/devcontainer.json` +
`docker-compose.yml` define an environment that GitHub Codespaces can run
identically in the cloud. This skill's four scripts create/reuse a
Codespace, sync your local working tree into it, run build/test commands on
it over SSH, and stop or delete it when you're done — giving you a real
devcontainer without local Docker.

## When to Use This Skill

- You're in a sandbox without Docker/devcontainer support (e.g. Codex
  Tasks) and need to build or test this ROS 2 workspace.
- `devcontainer exec` from the microVM-sandbox workflow is unavailable.
- You're explicitly asked to create, use, sync, or tear down a Codespace
  for this repo.

## Prerequisites

- `gh` CLI installed and authenticated with a token that has both `repo`
  and `codespace` OAuth scopes.
- This skill does **not** use the [gh-auth](../gh-auth/SKILL.md) skill's
  VS Code-credential-helper trick. `gh-auth` borrows a credential from a
  running VS Code Remote IPC host — Codex Tasks (this skill's primary
  target environment) is headless and has no such host to borrow from. The
  intended auth path here is a PAT/fine-grained token supplied via a
  `GH_TOKEN`/`GITHUB_TOKEN` environment secret configured in the Codex
  Tasks environment, not `gh auth login` and not `gh-auth`. Don't wire
  `gh-auth` into this skill.
- GitHub Codespaces clones this repo into `/workspaces/<repo-name>` (e.g.
  `/workspaces/Dr.QP`) and runs the devcontainer from there — it overrides
  the `/opt/ros/overlay_ws` bind mount this repo's `docker-compose.yml` uses
  for *local* devcontainers. All four scripts operate against that
  `/workspaces/<repo-name>` remote path, derived from the repo name via
  `__common.sh`'s `codespace_workspace_dir`.

## Bundled Scripts

- [codespace-ensure.sh](scripts/codespace-ensure.sh) — find-or-create the Codespace for a branch.
- [codespace-sync.sh](scripts/codespace-sync.sh) — sync the local working tree onto the Codespace.
- [codespace-exec.sh](scripts/codespace-exec.sh) — run a command on the Codespace via SSH.
- [codespace-teardown.sh](scripts/codespace-teardown.sh) — stop (default) or `--delete` the Codespace.

(`scripts/__common.sh` holds shared helpers sourced by the four scripts above; it is not a standalone entry point.)

## Workflow 1: Auth Preflight

Every script above calls `require_codespace_auth` (in `__common.sh`)
internally before doing anything else, so you don't need a separate
preflight step before using any of them. If you want to check auth in
isolation first, run the same read-only check they use under the hood:

```bash
gh codespace list --json name -q 'length'
```

Expect one of two distinct failure modes (see `require_codespace_auth` in
[__common.sh](scripts/__common.sh) for the exact logic):

- **Exit 2** — plain not-authenticated. `gh auth status` will also report
  failure.
- **Exit 3** — authenticated but the token is missing the `codespace`
  scope. This is a real, distinct failure worth naming explicitly:
  `gh auth status` can report success while the token still lacks
  `codespace`, so a green `gh auth status` does not guarantee these
  scripts will work.

## Workflow 2: Ensure the Codespace Exists

```bash
.github/skills/remote-codespace-session/scripts/codespace-ensure.sh
```

Flags: `--branch <name>` (default: current branch), `--machine <name>`
(default `standardLinux32gb`), `--idle-timeout <dur>` (default `30m`),
`--retention-period <dur>` (default `24h`; Go-style duration, `h`/`m`/`s`
only — no `d` — max `720h`), `--poll-interval <seconds>`
(default `10`), `--poll-timeout <seconds>` (default `300`), `--dry-run`.

Exit codes: `0` resolved (reused or created, or dry-run report printed),
`2` usage error/`gh` missing/plain auth failure, `3` under-scoped token,
`4` `gh codespace create` failed, `5` timed out waiting for a new
Codespace to reach `Available`.

**Cost guardrail — MUST, not a suggestion:** Before the *first* real
(non-dry-run) invocation of `codespace-ensure.sh` in a session where no
Codespace is already known to exist for this branch, run it with
`--dry-run` first, show the user the exact machine type / idle-timeout /
retention-period / branch that would be requested (this creates a
billable cloud resource), and get explicit confirmation before re-running
without `--dry-run`. If the `--dry-run` output reports `ACTION=reuse` (an
existing Codespace was found for this branch), skip the confirmation —
reuse doesn't spin up a new billable resource, so the guardrail doesn't
apply.

```bash
.github/skills/remote-codespace-session/scripts/codespace-ensure.sh --dry-run
```

## Workflow 3: Sync Local Changes Into the Codespace

```bash
.github/skills/remote-codespace-session/scripts/codespace-sync.sh
```

No flags besides `-h`/`--help`. The script picks the sync method for you:
if the working tree is clean, it pushes the branch and syncs the
Codespace's checkout via `git fetch` + `git reset --hard` over SSH; if the
tree is dirty (uncommitted or untracked changes), it rsyncs the working
tree directly over SSH (respecting `.gitignore`, excluding `.git`). Call
it before every remote build/test — it figures out the right method
itself.

Exit codes: `0` synced (either path), `2` usage error/missing Codespace
name/SSH config generation or parsing failure, `3` git push failed, `4`
rsync failed, `5` remote git fetch/checkout/reset over SSH failed.

## Workflow 4: Run Build/Test Remotely

```bash
.github/skills/remote-codespace-session/scripts/codespace-exec.sh <command> [args...]
```

This reuses the exact command vocabulary already documented in
[ros2-workspace-build](../ros2-workspace-build/SKILL.md) and
[ros2-workspace-testing](../ros2-workspace-testing/SKILL.md) — don't
duplicate that content, just translate a command from those skills into a
`codespace-exec.sh` call:

```bash
.github/skills/remote-codespace-session/scripts/codespace-exec.sh \
  colcon build --symlink-install --event-handlers console_cohesion+ \
  --packages-up-to <package_name>
```

Do **not** prefix commands with `scripts/with-ros-env.sh` yourself —
`codespace-exec.sh` already runs `scripts/with-ros-env.sh <command>
[args...]` on the Codespace for you. Everything after the script name is
forwarded verbatim as the remote command, including `-h`/`--help` (this
script has no help flag of its own to intercept it).

stdout/stderr stream back directly, and the script's own exit code is
exactly the remote command's exit code: `0` only if the remote command
succeeded, `2` if no command was given / `gh` missing / plain auth
failure / missing Codespace name, `3` if the token is under-scoped. Any
other non-zero exit is a normal build/test failure signal passed through
from the remote command, not a script error.

## Workflow 5: Retrieve Artifacts

There's no dedicated script for this — it's a plain reverse rsync using
the same SSH config `codespace-sync.sh` generates at
`./.tmp/codespace-ssh-config`:

```bash
host="$(awk '/^Host /{print $2; exit}' ./.tmp/codespace-ssh-config)"
# Remote path is /workspaces/<repo-name> (GitHub Codespaces' checkout root).
rsync -az -e "ssh -F ./.tmp/codespace-ssh-config" \
  "${host}:/workspaces/Dr.QP/log/latest_test/" ./log/latest_test/
```

`./.tmp/codespace-ssh-config` only exists after `codespace-sync.sh` has
run at least once via its rsync path (the git-push path doesn't generate
it). If it's missing, run `codespace-sync.sh` against a dirty tree (see
[Workflow 3](#workflow-3-sync-local-changes-into-the-codespace)) to create
it.

## Workflow 6: Teardown

```bash
.github/skills/remote-codespace-session/scripts/codespace-teardown.sh
.github/skills/remote-codespace-session/scripts/codespace-teardown.sh --delete
```

Default stops the Codespace (`gh codespace stop`) — halts compute
billing, keeps storage so a later `codespace-ensure.sh` can reuse it,
leaves `./.tmp/codespace-name` and `./.tmp/codespace-ssh-config` in place.
`--delete` fully deletes it (`gh codespace delete --force`) and removes
both of those `.tmp` files.

Exit codes: `0` stopped or deleted successfully, `2` usage error/`gh`
missing/plain auth failure/no Codespace name recorded, `3` under-scoped
token **or** the `gh codespace stop`/`delete` command itself failed —
check the printed `ERROR` message to tell which one occurred.

**Default policy:** stop (not delete) at the end of a session. Only pass
`--delete` when the user explicitly asks for full teardown, or when a
Codespace needs to be recreated from a corrupted state.

## Remote-Edit Feasibility (Not the Primary Workflow)

`gh codespace ssh -c <name> -- <command>` forwards stdin, so ad hoc remote
file writes are technically possible, e.g.:

```bash
gh codespace ssh -c <name> -- 'cat > path/to/file' <<'EOF'
...
EOF
```

This is an escape hatch for one-off remote-only debugging, not the
primary editing model. Keep editing your own local git working tree as
the source of truth and sync the diff over with `codespace-sync.sh`
before each remote run — that keeps `git status` meaningful and avoids
maintaining two divergent tree copies.
