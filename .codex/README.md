# Codex Customizations

This directory mirrors the repository's GitHub Copilot customizations so they
are discoverable in Codex.

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
