import os
from pathlib import Path
import shutil
import subprocess
import uuid

import pytest

REPO_ROOT = Path(__file__).parents[2]
SCRIPT = REPO_ROOT / '.devcontainer/scripts/claude-remote-control-session.sh'
ID_FILE_NAME = 'remote-control-session-id'
STORED_ID = '11111111-2222-4333-8444-555555555555'
# Logs each call's arguments and fails `--resume` with $CLAUDE_TEST_RESUME_EXIT.
FAKE_CLAUDE = (
    '#!/bin/sh\n'
    'printf "%s\\n" "$*" >>"$CLAUDE_TEST_LOG"\n'
    'case "$*" in *--resume*) exit "$CLAUDE_TEST_RESUME_EXIT" ;; esac\n'
)


@pytest.fixture
def test_dir():
    path = REPO_ROOT / '.tmp' / f'claude-remote-control-session-{uuid.uuid4().hex}'
    path.mkdir(parents=True)
    yield path
    shutil.rmtree(path)


def run_session(
    test_dir: Path, *args: str, stored_id: str | None = None, resume_exit: int = 0
) -> tuple[list[str], Path]:
    config_dir = test_dir / 'claude-config'
    config_dir.mkdir()
    id_file = config_dir / ID_FILE_NAME
    if stored_id is not None:
        id_file.write_text(f'{stored_id}\n')
    fake_bin = test_dir / 'bin'
    fake_bin.mkdir()
    (fake_bin / 'claude').write_text(FAKE_CLAUDE)
    (fake_bin / 'claude').chmod(0o755)
    log = test_dir / 'claude.log'
    env = os.environ | {
        'PATH': f'{fake_bin}:{os.environ["PATH"]}',
        'CLAUDE_CONFIG_DIR': str(config_dir),
        'CLAUDE_TEST_LOG': str(log),
        'CLAUDE_TEST_RESUME_EXIT': str(resume_exit),
    }
    result = subprocess.run([SCRIPT, *args], env=env, text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    return log.read_text().splitlines(), id_file


def stored(id_file: Path) -> str:
    return id_file.read_text().strip()


def test_first_start_pins_a_new_conversation(test_dir):
    calls, id_file = run_session(test_dir, 'agent-dev1')
    new_id = stored(id_file)
    assert str(uuid.UUID(new_id)) == new_id
    assert calls == [f'--remote-control agent-dev1 --session-id {new_id}']


def test_resumes_the_stored_conversation(test_dir):
    calls, id_file = run_session(test_dir, 'agent-dev1', stored_id=STORED_ID)
    assert calls == [f'--remote-control agent-dev1 --resume {STORED_ID}']
    assert stored(id_file) == STORED_ID


def test_failed_resume_pins_a_new_conversation(test_dir):
    calls, id_file = run_session(test_dir, 'agent-dev1', stored_id=STORED_ID, resume_exit=1)
    new_id = stored(id_file)
    assert new_id != STORED_ID
    assert calls == [
        f'--remote-control agent-dev1 --resume {STORED_ID}',
        f'--remote-control agent-dev1 --session-id {new_id}',
    ]


def test_without_a_name_runs_unnamed_remote_control(test_dir):
    calls, id_file = run_session(test_dir, stored_id=STORED_ID, resume_exit=1)
    assert calls == [
        f'--remote-control --resume {STORED_ID}',
        f'--remote-control --session-id {stored(id_file)}',
    ]
