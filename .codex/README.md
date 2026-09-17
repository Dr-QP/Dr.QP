# Codex Customizations

This directory adapts the repository's Claude-native AI tooling (canonical
source in `.claude/`) so it is discoverable in Codex:

- `skills/` — symlink to `../.claude/skills/`

Edit the `.claude/` sources, not this directory.

General-purpose skills and agents are not kept in this repository. They are
installed from the [`agentdev`](https://github.com/plume-works/agent-devcontainer)
catalog by the devcontainer's post-create hooks
(`.devcontainer/scripts/reinstall-agentdev-codex.sh`) and are invoked under the
`agentdev:` prefix.

## Codex Cloud environment setup

Use [`setup-codex-cloud.sh`](setup-codex-cloud.sh) as the Codex Cloud setup
script for this repository. It prepares the host-side tools that agents need:

- installs GitHub CLI (`gh`) when it is missing; and
- checks whether `gh` can authenticate through an existing login, `GH_TOKEN`, or
  `GITHUB_TOKEN`.

Codex Cloud tasks have no ROS environment, so this script does not set up or
run ROS build/test/lint/launch commands.

Keep GitHub credentials in Codex Cloud environment secrets instead of committing
them. Prefer `GH_TOKEN` for GitHub CLI automation. If no token is present, the
setup script leaves `gh` installed and reports the authentication blocker.
