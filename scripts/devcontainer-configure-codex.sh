#!/usr/bin/env bash
set -euo pipefail

# This command runs inside the devcontainer. User-scope settings therefore live
# in the container's /root/.codex volume rather than in the repository, so
# Codex Cloud and host sessions retain their normal sandboxing defaults.
codex config set --scope user sandbox_mode danger-full-access
codex config set --scope user approval_policy never
