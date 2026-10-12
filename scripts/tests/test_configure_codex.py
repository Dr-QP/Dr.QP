import os
from pathlib import Path
import stat
import subprocess
import tomllib

import pytest

SCRIPT = Path(__file__).parents[2] / '.devcontainer/scripts/configure-codex.py'
MANAGED = {'sandbox_mode': 'danger-full-access', 'approval_policy': 'never'}
MARKED_CONFIG = """\
# >>> codebase-memory-mcp MCP >>>
[mcp_servers.codebase-memory-mcp]
command = "/root/.local/bin/codebase-memory-mcp"  # trailing note
# <<< codebase-memory-mcp MCP <<<

model = "gpt-5"
"""


@pytest.fixture
def codex_home(tmp_path: Path) -> Path:
    return tmp_path / 'codex'


def run_script(codex_home: Path) -> None:
    subprocess.run([SCRIPT], env=os.environ | {'CODEX_HOME': str(codex_home)}, check=True)


def mode(path: Path) -> int:
    return stat.S_IMODE(path.stat().st_mode)


def test_absent_config_is_created_with_both_keys_and_private_modes(codex_home: Path):
    run_script(codex_home)

    config = codex_home / 'config.toml'
    assert tomllib.loads(config.read_text()) == MANAGED
    assert mode(codex_home) == 0o700
    assert mode(config) == 0o600


def test_other_keys_tables_and_marker_comments_survive_unchanged(codex_home: Path):
    codex_home.mkdir()
    config = codex_home / 'config.toml'
    config.write_text(MARKED_CONFIG)

    run_script(codex_home)

    managed_lines = ''.join(f'{key} = "{value}"\n' for key, value in MANAGED.items())
    assert config.read_text() == managed_lines + '\n' + MARKED_CONFIG


def test_different_values_are_overwritten_in_place(codex_home: Path):
    codex_home.mkdir()
    config = codex_home / 'config.toml'
    config.write_text(
        'sandbox_mode = "workspace-write"\n'
        'approval_policy = "on-request"\n\n'
        '[profiles.ci]\nmodel = "gpt-5"\n'
    )

    run_script(codex_home)

    parsed = tomllib.loads(config.read_text())
    assert {key: parsed[key] for key in MANAGED} == MANAGED
    assert parsed['profiles'] == {'ci': {'model': 'gpt-5'}}


def test_second_run_leaves_content_unchanged(codex_home: Path):
    codex_home.mkdir()
    config = codex_home / 'config.toml'
    config.write_text(MARKED_CONFIG)
    run_script(codex_home)
    first = config.read_bytes()

    run_script(codex_home)

    assert config.read_bytes() == first


def test_unparseable_config_fails_and_is_left_in_place(codex_home: Path):
    codex_home.mkdir()
    config = codex_home / 'config.toml'
    config.write_text('model = \n')

    with pytest.raises(subprocess.CalledProcessError):
        run_script(codex_home)

    assert config.read_text() == 'model = \n'
    assert [path.name for path in codex_home.iterdir()] == ['config.toml']
