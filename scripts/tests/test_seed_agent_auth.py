import json
import os
from pathlib import Path
import stat
import subprocess

REPO_ROOT = Path(__file__).parents[2]
PREPARE_SCRIPT = REPO_ROOT / '.devcontainer/scripts/prepare-agent-auth-seed.sh'
SEED_SCRIPT = REPO_ROOT / '.devcontainer/scripts/seed-agent-auth.sh'
INIT_SCRIPT = REPO_ROOT / '.devcontainer/devcontainer-init.sh'
POST_CREATE = REPO_ROOT / '.devcontainer/scripts/postCreateCommand.sh'
POST_START = REPO_ROOT / '.devcontainer/scripts/postStartCommand.sh'
COMPOSE = REPO_ROOT / '.devcontainer/docker-compose.yml'


def run_prepare(
    tmp_path: Path, *, claude: str = '', codex: str = ''
) -> tuple[subprocess.CompletedProcess[str], Path]:
    seed_dir = tmp_path / 'seed'
    env = os.environ | {
        'AGENTDEV_CLAUDE_JSON': claude,
        'AGENTDEV_CODEX_JSON': codex,
        'AGENTDEV_AUTH_SEED_DIR': str(seed_dir),
    }
    result = subprocess.run([PREPARE_SCRIPT], env=env, text=True, capture_output=True)
    return result, seed_dir


def run_seeder(
    tmp_path: Path, *, claude: str = '', codex: str = ''
) -> tuple[subprocess.CompletedProcess[str], Path, Path, Path]:
    seed_dir = tmp_path / 'seed'
    seed_dir.mkdir(mode=0o700)
    if claude:
        (seed_dir / 'claude.json').write_text(claude)
    if codex:
        (seed_dir / 'codex.json').write_text(codex)
    claude_path = tmp_path / 'auth' / 'claude' / '.credentials.json'
    codex_path = tmp_path / 'auth' / 'codex' / 'auth.json'
    env = os.environ | {
        'AGENTDEV_AUTH_SEED_DIR': str(seed_dir),
        'AGENTDEV_CLAUDE_AUTH_PATH': str(claude_path),
        'AGENTDEV_CODEX_AUTH_PATH': str(codex_path),
    }
    result = subprocess.run([SEED_SCRIPT], env=env, text=True, capture_output=True)
    return result, seed_dir, claude_path, codex_path


def test_prepare_writes_private_transfer_files(tmp_path):
    claude = json.dumps({'claudeAiOauth': {'accessToken': 'claude-secret'}})
    codex = json.dumps({'auth_mode': 'chatgpt', 'tokens': {'refresh_token': 'codex-secret'}})

    result, seed_dir = run_prepare(tmp_path, claude=claude, codex=codex)

    assert result.returncode == 0, result.stderr
    assert json.loads((seed_dir / 'claude.json').read_text()) == json.loads(claude)
    assert json.loads((seed_dir / 'codex.json').read_text()) == json.loads(codex)
    assert stat.S_IMODE(seed_dir.stat().st_mode) == 0o700
    assert stat.S_IMODE((seed_dir / 'claude.json').stat().st_mode) == 0o600
    assert stat.S_IMODE((seed_dir / 'codex.json').stat().st_mode) == 0o600
    assert 'claude-secret' not in result.stdout + result.stderr
    assert 'codex-secret' not in result.stdout + result.stderr


def test_prepare_rejects_invalid_json_without_leaking_value(tmp_path):
    result, seed_dir = run_prepare(tmp_path, claude='not-json')

    assert result.returncode != 0
    assert not (seed_dir / 'claude.json').exists()
    assert 'not-json' not in result.stdout + result.stderr


def test_prepare_unset_value_removes_stale_transfer_file(tmp_path):
    seed_dir = tmp_path / 'seed'
    seed_dir.mkdir()
    (seed_dir / 'claude.json').write_text('{"stale": true}')

    result, _ = run_prepare(tmp_path)

    assert result.returncode == 0, result.stderr
    assert not (seed_dir / 'claude.json').exists()


def test_seeds_both_credentials_and_consumes_transfer_files(tmp_path):
    claude = json.dumps({'claudeAiOauth': {'accessToken': 'claude-secret'}})
    codex = json.dumps({'auth_mode': 'chatgpt', 'tokens': {'refresh_token': 'codex-secret'}})

    result, seed_dir, claude_path, codex_path = run_seeder(tmp_path, claude=claude, codex=codex)

    assert result.returncode == 0, result.stderr
    assert json.loads(claude_path.read_text()) == json.loads(claude)
    assert json.loads(codex_path.read_text()) == json.loads(codex)
    assert stat.S_IMODE(claude_path.parent.stat().st_mode) == 0o700
    assert stat.S_IMODE(codex_path.parent.stat().st_mode) == 0o700
    assert stat.S_IMODE(claude_path.stat().st_mode) == 0o600
    assert stat.S_IMODE(codex_path.stat().st_mode) == 0o600
    assert not (seed_dir / 'claude.json').exists()
    assert not (seed_dir / 'codex.json').exists()
    assert 'claude-secret' not in result.stdout + result.stderr
    assert 'codex-secret' not in result.stdout + result.stderr


def test_preserves_existing_credentials_but_fixes_mode_and_consumes_seed(tmp_path):
    claude_path = tmp_path / 'auth' / 'claude' / '.credentials.json'
    codex_path = tmp_path / 'auth' / 'codex' / 'auth.json'
    for path in (claude_path, codex_path):
        path.parent.mkdir(parents=True)
        path.write_text('{"existing": true}')
        path.chmod(0o644)

    result, seed_dir, _, _ = run_seeder(
        tmp_path,
        claude='{"replacement": "claude-secret"}',
        codex='{"replacement": "codex-secret"}',
    )

    assert result.returncode == 0, result.stderr
    assert json.loads(claude_path.read_text()) == {'existing': True}
    assert json.loads(codex_path.read_text()) == {'existing': True}
    assert stat.S_IMODE(claude_path.stat().st_mode) == 0o600
    assert stat.S_IMODE(codex_path.stat().st_mode) == 0o600
    assert not (seed_dir / 'claude.json').exists()
    assert not (seed_dir / 'codex.json').exists()


def test_fixes_existing_credential_mode_without_seed(tmp_path):
    claude_path = tmp_path / 'auth' / 'claude' / '.credentials.json'
    claude_path.parent.mkdir(parents=True)
    claude_path.write_text('{"existing": true}')
    claude_path.chmod(0o644)

    result, _, _, _ = run_seeder(tmp_path)

    assert result.returncode == 0, result.stderr
    assert json.loads(claude_path.read_text()) == {'existing': True}
    assert stat.S_IMODE(claude_path.stat().st_mode) == 0o600


def test_seeder_rejects_invalid_transfer_file(tmp_path):
    result, seed_dir, claude_path, _ = run_seeder(tmp_path, claude='not-json')

    assert result.returncode != 0
    assert not claude_path.exists()
    assert not (seed_dir / 'claude.json').exists()
    assert 'not-json' not in result.stdout + result.stderr


def test_compose_mounts_seed_directory_without_secret_environment():
    compose = COMPOSE.read_text()

    assert 'AGENTDEV_CLAUDE_JSON' not in compose
    assert 'AGENTDEV_CODEX_JSON' not in compose
    assert 'AGENTDEV_AUTH_SEED_DIR' in compose
    assert ':/run/agentdev-auth-seed:rw' in compose


def test_lifecycle_calls_prepare_then_consume():
    init = INIT_SCRIPT.read_text()
    post_create = POST_CREATE.read_text()

    assert '"$script_dir/scripts/prepare-agent-auth-seed.sh"' in init
    assert '"$script_dir/seed-agent-auth.sh"' in post_create
    # Init rewrites transfer files on every start, so start must consume them too.
    assert '"$script_dir/seed-agent-auth.sh"' in POST_START.read_text()
