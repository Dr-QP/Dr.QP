#!/usr/bin/env bash
# Pre-answers Claude's first-run prompts so an autostarted Remote Control session
# needs no terminal: onboarding, Remote Control confirmation, workspace trust, and
# project MCP approval. Spec: devcontainer-agent-auth.
set -euo pipefail

if [[ "${AGENTDEV_CLAUDE_AUTOSTART:-}" != "1" ]]; then
  exit 0
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="${DEV_WORKSPACE_FOLDER:-$(cd "$script_dir/../.." && pwd)}"
state_file="${AGENTDEV_CLAUDE_STATE_PATH:-$HOME/.claude.json}"

python3 - "$state_file" "$workspace" <<'EOF'
import json
import os
import sys
import tempfile

state_path, workspace = sys.argv[1], sys.argv[2]


def update_json(path, mutate):
    # Resolve first so a symlink into a persistent volume stays a symlink.
    path = os.path.realpath(path)
    try:
        with open(path) as source:
            data = json.load(source)
    except FileNotFoundError:
        data = {}
    mutate(data)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fd, temp = tempfile.mkstemp(dir=os.path.dirname(path), prefix=".preapprove.")
    with os.fdopen(fd, "w") as target:
        json.dump(data, target, indent=2)
        target.write("\n")
    os.chmod(temp, 0o600)
    os.replace(temp, path)


def approve_state(state):
    state["hasCompletedOnboarding"] = True
    state["remoteDialogSeen"] = True
    project = state.setdefault("projects", {}).setdefault(workspace, {})
    project["hasTrustDialogAccepted"] = True


def approve_mcp(settings):
    # Keeps disabledMcpjsonServers; see spec/devcontainer-agent-auth.
    settings["enableAllProjectMcpServers"] = True


update_json(state_path, approve_state)
update_json(os.path.join(workspace, ".claude", "settings.local.json"), approve_mcp)
EOF

echo "Pre-approved Claude onboarding, Remote Control, trust, and MCP for $workspace."
