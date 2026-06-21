# Codex Customizations

This directory mirrors the repository's GitHub Copilot customizations so they
are discoverable in Codex.

## Codex Cloud environment setup

Use [`setup-codex-cloud.sh`](setup-codex-cloud.sh) as the Codex Cloud setup
script for this repository. It prepares the host-side tools that agents need
before entering the ROS devcontainer:

- installs GitHub CLI (`gh`) when it is missing;
- installs the VS Code Dev Containers CLI (`devcontainer`) when it is missing;
- starts or verifies the host Docker daemon;
- checks whether `gh` can authenticate through an existing login, `GH_TOKEN`, or
  `GITHUB_TOKEN`; and
- runs `devcontainer up --workspace-folder /workspace` with the host Docker
  socket mounted into the container.

After setup, run commands inside the container with:

```bash
devcontainer exec --workspace-folder /workspace bash -lc '<command>'
```

Run ROS build, test, lint, and launch commands through the repository wrapper
inside that `devcontainer exec` call:

```bash
devcontainer exec --workspace-folder /workspace bash -lc \
  'scripts/with-ros-env.sh <ros command>'
```

Keep GitHub credentials in Codex Cloud environment secrets instead of committing
them. Prefer `GH_TOKEN` for GitHub CLI automation. If no token is present, the
setup script leaves `gh` installed and reports the authentication blocker.
