import os
from pathlib import Path
import shutil
import stat
import subprocess
import time
import uuid

import pytest

REPO_ROOT = Path(__file__).parents[2]
SCRIPT = REPO_ROOT / '.devcontainer/scripts/opencode-web-start.sh'
SERVE = 'opencode serve --port 4096 --hostname'
EXPOSED_COMMAND = f'exec env -u DISPLAY {SERVE} 0.0.0.0'
PASSWORD_FILE = '.local/share/opencode/agentdev-server-password'
PROXY_URI = 'https://{{port}}--repo--ws--me.example.com'
CODER_LINK = 'https://4096--repo--ws--me.example.com'
LOCAL_LINK = 'http://127.0.0.1:4096'


def make_test_dir() -> Path:
    path = REPO_ROOT / '.tmp' / f'opencode-web-{uuid.uuid4().hex}'
    path.mkdir(parents=True)
    return path


def local_command(home: Path) -> str:
    """Return the pane command that reads the generated password file itself."""
    return (
        'exec bash -c \'OPENCODE_SERVER_PASSWORD="$(cat "$1")" exec env -u DISPLAY '
        f"{SERVE} 127.0.0.1' opencode-web {home / PASSWORD_FILE}"
    )


def run_with_fake_tmux(
    *,
    home: Path,
    password: str | None = None,
    session_exists: bool = False,
    missing: str | None = None,
    proxy_uri: str | None = None,
) -> tuple[subprocess.CompletedProcess[str], str]:
    """Run the script beside a fake tmux that logs its arguments."""
    test_dir = make_test_dir()
    log = test_dir / 'tmux.log'
    fake_bin = test_dir / 'bin'
    fake_bin.mkdir()
    fakes = {
        'opencode': '#!/bin/sh\nexit 0\n',
        'tmux': '#!/bin/sh\n'
        'printf "%s\\n" "$*" >>"$TMUX_TEST_LOG"\n'
        'case "$*" in *has-session*) [ "$TMUX_HAS_SESSION" = 1 ] ;; esac\n',
    }
    for name, body in fakes.items():
        if name == missing:
            continue
        (fake_bin / name).write_text(body)
        (fake_bin / name).chmod(0o755)
    # Only the fakes and the coreutils the script needs, so a real tmux cannot leak in.
    tools = test_dir / 'tools'
    tools.mkdir()
    for tool in ('base64', 'bash', 'env', 'head', 'mkdir', 'printf', 'tr'):
        (tools / tool).symlink_to(shutil.which(tool))
    env = {
        'PATH': f'{fake_bin}:{tools}',
        'HOME': str(home),
        'DEV_WORKSPACE_FOLDER': '/workspaces/example',
        'TMUX_HAS_SESSION': '1' if session_exists else '0',
        'TMUX_TEST_LOG': str(log),
    }
    if proxy_uri is not None:
        env['VSCODE_PROXY_URI'] = proxy_uri
    if password is not None:
        env['OPENCODE_SERVER_PASSWORD'] = password
    try:
        result = subprocess.run(
            [shutil.which('bash'), SCRIPT], env=env, text=True, capture_output=True
        )
        return result, log.read_text() if log.exists() else ''
    finally:
        shutil.rmtree(test_dir)


@pytest.fixture
def home():
    path = make_test_dir()
    yield path
    shutil.rmtree(path)


def assert_dedicated_socket(calls: str) -> None:
    lines = calls.splitlines()
    assert lines
    assert all(line.startswith('-L opencode ') for line in lines), calls


def test_binds_localhost_with_a_generated_password(home):
    result, calls = run_with_fake_tmux(home=home)
    assert result.returncode == 0, result.stderr
    assert 'new-session -d -s opencode -c /workspaces/example' in calls
    assert calls.splitlines()[-1].endswith(local_command(home))
    assert_dedicated_socket(calls)


def test_generates_the_password_file_once_with_owner_only_access(home):
    first, _ = run_with_fake_tmux(home=home)
    assert first.returncode == 0, first.stderr
    password_file = home / PASSWORD_FILE
    generated = password_file.read_text()
    assert len(generated) >= 32
    assert stat.S_IMODE(password_file.stat().st_mode) == 0o600
    second, _ = run_with_fake_tmux(home=home)
    assert second.returncode == 0, second.stderr
    assert password_file.read_text() == generated


@pytest.mark.parametrize('session_exists', [False, True])
def test_never_shows_the_generated_password_and_names_its_file(home, session_exists):
    run_with_fake_tmux(home=home)
    generated = (home / PASSWORD_FILE).read_text()
    result, calls = run_with_fake_tmux(home=home, session_exists=session_exists)
    assert result.returncode == 0, result.stderr
    for text in (calls, result.stdout, result.stderr):
        assert generated not in text
    assert str(home / PASSWORD_FILE) in result.stdout


def test_binds_all_interfaces_with_a_password(home):
    result, calls = run_with_fake_tmux(home=home, password='s3cret')
    assert result.returncode == 0, result.stderr
    assert calls.splitlines()[-1].endswith(EXPOSED_COMMAND)
    assert 's3cret' not in calls + result.stdout + result.stderr
    assert not (home / PASSWORD_FILE).exists()
    assert PASSWORD_FILE not in result.stdout


def test_empty_password_uses_the_generated_password(home):
    result, calls = run_with_fake_tmux(home=home, password='')
    assert result.returncode == 0, result.stderr
    assert calls.splitlines()[-1].endswith(local_command(home))


def test_reuses_a_live_session(home):
    result, calls = run_with_fake_tmux(home=home, session_exists=True)
    assert result.returncode == 0, result.stderr
    assert 'new-session' not in calls
    assert_dedicated_socket(calls)


@pytest.mark.parametrize('missing', ['opencode', 'tmux'])
def test_skips_when_a_command_is_missing(home, missing):
    result, calls = run_with_fake_tmux(home=home, missing=missing)
    assert result.returncode == 0, result.stderr
    assert calls == ''
    assert missing in result.stdout + result.stderr


@pytest.mark.parametrize('session_exists', [False, True])
@pytest.mark.parametrize('password', [None, 's3cret'])
def test_prints_the_coder_server_url(home, session_exists, password):
    result, _ = run_with_fake_tmux(
        home=home, proxy_uri=PROXY_URI, session_exists=session_exists, password=password
    )
    assert result.returncode == 0, result.stderr
    assert CODER_LINK in result.stdout.split()


def test_trailing_slash_in_the_port_url_adds_no_empty_segment(home):
    result, _ = run_with_fake_tmux(home=home, proxy_uri=f'{PROXY_URI}/')
    assert result.returncode == 0, result.stderr
    assert CODER_LINK in result.stdout.split()


@pytest.mark.parametrize('proxy_uri', [None, ''])
def test_falls_back_to_localhost_outside_coder(home, proxy_uri):
    result, _ = run_with_fake_tmux(home=home, proxy_uri=proxy_uri)
    assert result.returncode == 0, result.stderr
    assert LOCAL_LINK in result.stdout.split()


@pytest.mark.parametrize('missing', ['opencode', 'tmux'])
def test_skip_prints_no_link(home, missing):
    result, _ = run_with_fake_tmux(home=home, missing=missing, proxy_uri=PROXY_URI)
    assert result.returncode == 0, result.stderr
    assert CODER_LINK not in result.stdout + result.stderr


def run_with_real_tmux(test_dir: Path, password: str | None) -> str:
    """Start the script under real tmux with a fake opencode; return what it recorded."""
    fake_bin = test_dir / 'bin'
    fake_bin.mkdir()
    result_file = test_dir / 'result'
    (fake_bin / 'opencode').write_text(
        '#!/bin/sh\n'
        'printf "%s|%s|%s" "$*" "${OPENCODE_SERVER_PASSWORD:-}" "${DISPLAY:-none}"'
        ' >"$OPENCODE_TEST_RESULT"\n'
        'sleep 10\n'
    )
    (fake_bin / 'opencode').chmod(0o755)
    env = os.environ | {
        'PATH': f'{fake_bin}:{os.environ["PATH"]}',
        'HOME': str(test_dir),
        'TMUX_TMPDIR': str(REPO_ROOT / '.tmp'),
        'OPENCODE_TEST_RESULT': str(result_file),
        'DISPLAY': ':100',
        'DEV_WORKSPACE_FOLDER': str(REPO_ROOT),
    }
    env.pop('TMUX', None)
    env.pop('OPENCODE_SERVER_PASSWORD', None)
    if password is not None:
        env['OPENCODE_SERVER_PASSWORD'] = password
    try:
        result = subprocess.run([SCRIPT], env=env, text=True, capture_output=True)
        assert result.returncode == 0, result.stderr
        for _ in range(30):
            if result_file.exists() and result_file.read_text():
                break
            time.sleep(0.1)
        return result_file.read_text()
    finally:
        subprocess.run(['tmux', '-L', 'opencode', 'kill-server'], env=env, capture_output=True)


@pytest.mark.skipif(shutil.which('tmux') is None, reason='tmux is not installed')
def test_real_tmux_passes_the_password_and_drops_display(home):
    recorded = run_with_real_tmux(home, 'tmux-secret')
    assert recorded == 'serve --port 4096 --hostname 0.0.0.0|tmux-secret|none'


@pytest.mark.skipif(shutil.which('tmux') is None, reason='tmux is not installed')
def test_real_tmux_serves_with_the_generated_password(home):
    recorded = run_with_real_tmux(home, None)
    generated = (home / PASSWORD_FILE).read_text()
    assert recorded == f'serve --port 4096 --hostname 127.0.0.1|{generated}|none'
