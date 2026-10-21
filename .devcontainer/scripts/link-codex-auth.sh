#!/usr/bin/env bash
set -euo pipefail

# Codex has no equivalent of Claude Code's CLAUDE_SECURESTORAGE_CONFIG_DIR:
# CODEX_HOME is the single root for all of Codex's state (config, auth, logs,
# sessions, plugins), so only auth.json itself can be shared, not a whole
# directory. Persist it as a plain file inside the shared agentdev-agents-auth volume and
# symlink it into place, the same trick postCreateCommand.sh already uses for
# ~/.claude.json.
#
# `codex login` and token refresh write through this symlink in place (verified
# empirically: the shared target's content updates, the symlink survives). But
# `codex logout` unlink()s the local path outright rather than clearing it in
# place, destroying the symlink and leaving the shared file untouched. Called
# again from postCreateCommand.sh and postStartCommand.sh, this script repairs
# the symlink on the next container (re)start; a mid-session logout->login
# cycle before then creates an unshared local auth.json until it does.

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=runtime-paths.sh
source "$script_dir/runtime-paths.sh"

codex_auth_dir="$agentdev_auth_dir/codex"
codex_auth_target="$codex_auth_dir/auth.json"
codex_auth_local="$agentdev_codex_home/auth.json"

mkdir -p "$codex_auth_dir" "$agentdev_codex_home"
chmod 700 "$codex_auth_dir"

if [[ -f "$codex_auth_local" && ! -L "$codex_auth_local" ]]; then
    mv "$codex_auth_local" "$codex_auth_target"
    chmod 600 "$codex_auth_target"
fi

ln -sf "$codex_auth_target" "$codex_auth_local"
