import json
import os
from pathlib import Path
import stat
import subprocess

REPO_ROOT = Path(__file__).parents[2]
SCRIPT = REPO_ROOT / '.devcontainer/scripts/preapprove-claude-workspace.sh'
POST_CREATE = REPO_ROOT / '.devcontainer/scripts/postCreateCommand.sh'
AGENT_STATE = REPO_ROOT / '.devcontainer/scripts/setup-agent-state.sh'
RESPONDER_ACTION = REPO_ROOT / '.github/actions/run-claude-responder/action.yml'


def run_preapprove(
    tmp_path: Path, *, skip: str = ''
) -> tuple[subprocess.CompletedProcess[str], Path]:
    workspace = tmp_path / 'workspace'
    workspace.mkdir(exist_ok=True)
    env = {k: v for k, v in os.environ.items() if k != 'AGENTDEV_SKIP_CLAUDE_PREAPPROVE'} | {
        'AGENTDEV_SKIP_CLAUDE_PREAPPROVE': skip,
        'AGENTDEV_CLAUDE_STATE_PATH': str(tmp_path / '.claude.json'),
        'DEV_WORKSPACE_FOLDER': str(workspace),
    }
    result = subprocess.run([SCRIPT], env=env, text=True, capture_output=True)
    return result, workspace


def test_approves_state_and_mcp_preserving_existing_keys_and_denials(tmp_path):
    workspace = tmp_path / 'workspace'
    (workspace / '.claude').mkdir(parents=True)
    local_settings = workspace / '.claude' / 'settings.local.json'
    local_settings.write_text(json.dumps({'disabledMcpjsonServers': ['iwe'], 'keep': 1}))
    (tmp_path / '.claude.json').write_text(json.dumps({'mcpServers': {'cbm': {}}}))

    result, _ = run_preapprove(tmp_path)

    assert result.returncode == 0, result.stderr
    state = json.loads((tmp_path / '.claude.json').read_text())
    assert state['mcpServers'] == {'cbm': {}}
    assert state['hasCompletedOnboarding'] is True
    assert state['remoteDialogSeen'] is True
    assert state['projects'][str(workspace)]['hasTrustDialogAccepted'] is True
    assert stat.S_IMODE((tmp_path / '.claude.json').stat().st_mode) == 0o600
    assert json.loads(local_settings.read_text()) == {
        'disabledMcpjsonServers': ['iwe'],
        'keep': 1,
        'enableAllProjectMcpServers': True,
    }


def test_writes_through_state_symlink(tmp_path):
    volume_file = tmp_path / 'volume' / 'claude.json'
    volume_file.parent.mkdir()
    volume_file.write_text('{}')
    (tmp_path / '.claude.json').symlink_to(volume_file)

    result, _ = run_preapprove(tmp_path)

    assert result.returncode == 0, result.stderr
    assert (tmp_path / '.claude.json').is_symlink()
    assert json.loads(volume_file.read_text())['remoteDialogSeen'] is True


def test_runs_without_autostart(tmp_path):
    result, workspace = run_preapprove(tmp_path)

    assert result.returncode == 0, result.stderr
    assert json.loads((tmp_path / '.claude.json').read_text())['hasCompletedOnboarding'] is True
    assert (workspace / '.claude' / 'settings.local.json').exists()


def test_skips_when_opted_out(tmp_path):
    result, workspace = run_preapprove(tmp_path, skip='1')

    assert result.returncode == 0, result.stderr
    assert not (tmp_path / '.claude.json').exists()
    assert not (workspace / '.claude').exists()


def test_post_create_runs_after_state_symlink():
    assert '"$script_dir/setup-agent-state.sh" create' in POST_CREATE.read_text()
    create = AGENT_STATE.read_text().split('cmd_create() {', 1)[1]
    assert create.index('"$script_dir/preapprove-claude-workspace.sh"') > create.index(
        'link_claude_json'
    )


def test_ci_responder_opts_out():
    assert "AGENTDEV_SKIP_CLAUDE_PREAPPROVE: '1'" in RESPONDER_ACTION.read_text()
