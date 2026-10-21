import os
from pathlib import Path
import shutil
import subprocess

REPO_ROOT = Path(__file__).parents[2]
SCRIPT = REPO_ROOT / '.devcontainer/scripts/coder-agent-start.sh'
IMAGE_PRUNE = 'docker-image-prune.sh'
HELPER_SETUP = 'setup-gh-credential-helper.sh'
OPENCODE_WEB = 'opencode-web-start.sh'
REMOTE_CONTROL = 'claude-remote-control-start.sh'


def run_agent_start(
    tmp_path: Path,
    *,
    prune_exit: int = 0,
    helper_exit: int = 0,
    opencode_exit: int = 0,
    env: dict[str, str] | None = None,
) -> tuple[subprocess.CompletedProcess[str], list[str]]:
    """Run a copy of the script beside fakes that append their name to a log."""
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    script = scripts / SCRIPT.name
    shutil.copy2(SCRIPT, script)
    log = tmp_path / 'calls.log'
    fakes = {
        IMAGE_PRUNE: f'echo "{IMAGE_PRUNE}" >> "{log}"\nexit {prune_exit}\n',
        HELPER_SETUP: f'echo "{HELPER_SETUP} token=${{GITHUB_TOKEN:-}}" >> "{log}"\n'
        f'exit {helper_exit}\n',
        OPENCODE_WEB: f'echo "{OPENCODE_WEB} password=${{OPENCODE_SERVER_PASSWORD:-}}" >> "{log}"\n'
        f'exit {opencode_exit}\n',
        REMOTE_CONTROL: f'echo "{REMOTE_CONTROL}" >> "{log}"\n',
    }
    for name, body in fakes.items():
        fake = scripts / name
        fake.write_text('#!/usr/bin/env bash\n' + body)
        fake.chmod(0o755)
    result = subprocess.run(
        [script], env={'PATH': os.environ['PATH'], **(env or {})}, text=True, capture_output=True
    )
    calls = log.read_text().splitlines() if log.exists() else []
    return result, calls


def test_prune_helper_and_opencode_run_before_remote_control(tmp_path):
    result, calls = run_agent_start(tmp_path)
    assert result.returncode == 0, result.stderr
    assert [call.split()[0] for call in calls] == [
        IMAGE_PRUNE,
        HELPER_SETUP,
        OPENCODE_WEB,
        REMOTE_CONTROL,
    ]


def test_remote_control_starts_when_prune_fails(tmp_path):
    result, calls = run_agent_start(tmp_path, prune_exit=3)
    assert result.returncode == 0, result.stderr
    assert [call.split()[0] for call in calls] == [
        IMAGE_PRUNE,
        HELPER_SETUP,
        OPENCODE_WEB,
        REMOTE_CONTROL,
    ]
    assert result.stderr.strip()


def test_remote_control_starts_when_helper_setup_fails(tmp_path):
    result, calls = run_agent_start(tmp_path, helper_exit=3)
    assert result.returncode == 0, result.stderr
    assert calls[-1] == REMOTE_CONTROL
    assert result.stderr.strip()


def test_helper_setup_sees_caller_environment(tmp_path):
    result, calls = run_agent_start(tmp_path, env={'GITHUB_TOKEN': 'from-coder-secret'})
    assert result.returncode == 0, result.stderr
    assert calls[1] == f'{HELPER_SETUP} token=from-coder-secret'


def test_remote_control_starts_when_opencode_web_fails(tmp_path):
    result, calls = run_agent_start(tmp_path, opencode_exit=3)
    assert result.returncode == 0, result.stderr
    assert calls[-1] == REMOTE_CONTROL
    assert result.stderr.strip()


def test_opencode_web_sees_caller_environment(tmp_path):
    result, calls = run_agent_start(
        tmp_path, env={'OPENCODE_SERVER_PASSWORD': 'from-coder-secret'}
    )
    assert result.returncode == 0, result.stderr
    assert calls[2] == f'{OPENCODE_WEB} password=from-coder-secret'
