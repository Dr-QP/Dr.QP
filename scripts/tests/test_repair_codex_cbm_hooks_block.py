import os
from pathlib import Path
import stat
import subprocess
import tomllib

import pytest

SCRIPT = Path(__file__).parents[2] / '.devcontainer/scripts/repair-codex-cbm-hooks-block.py'
OPEN = '# >>> codebase-memory-mcp SessionStart >>>\n'
CLOSE = '# <<< codebase-memory-mcp SessionStart <<<\n'
MCP_BLOCK = """\
# >>> codebase-memory-mcp MCP >>>
[mcp_servers.codebase-memory-mcp]
command = "/bin/cbm"
env_vars = ["CBM_CACHE_DIR"]

[mcp_servers.codebase-memory-mcp.tools.index_status]
approval_mode = "approve"
# <<< codebase-memory-mcp MCP <<<
"""
OWNED_HOOKS = """\
[[hooks.SessionStart]]
matcher = "startup|resume|clear|compact"

[[hooks.SessionStart.hooks]]
type = "command"
command = "'/bin/cbm' hook-augment"
timeout = 5

[[hooks.SubagentStart]]
matcher = "*"

[[hooks.SubagentStart.hooks]]
type = "command"
command = "'/bin/cbm' hook-augment"
timeout = 5
"""
FOREIGN = """\
[hooks.state]

# kept with the table it describes
[hooks.state."agentdev:hooks.json:session_start:0:0"]
trusted_hash = "sha256:abc"

[plugins."agentdev@agent-devcontainer"]
enabled = true
"""
HEADER = 'model = "gpt-5"\n'


@pytest.fixture
def codex_home(tmp_path: Path) -> Path:
    home = tmp_path / 'codex'
    home.mkdir()
    return home


def run_script(codex_home: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [SCRIPT],
        env=os.environ | {'CODEX_HOME': str(codex_home)},
        check=True,
        capture_output=True,
        text=True,
    )


def write_config(codex_home: Path, content: str) -> Path:
    config = codex_home / 'config.toml'
    config.write_text(content)
    config.chmod(0o600)
    return config


def test_foreign_tables_move_below_the_closing_marker(codex_home: Path):
    original = HEADER + MCP_BLOCK + OPEN + OWNED_HOOKS + '\n' + FOREIGN + CLOSE
    config = write_config(codex_home, original)

    run_script(codex_home)

    repaired = config.read_text()
    assert repaired == HEADER + MCP_BLOCK + OPEN + OWNED_HOOKS + CLOSE + '\n' + FOREIGN
    assert tomllib.loads(repaired) == tomllib.loads(original)
    assert stat.S_IMODE(config.stat().st_mode) == 0o600


def test_foreign_tables_join_content_already_below_the_block(codex_home: Path):
    tail = '\n[tui]\nscreen_reader_detection_done = true\n'
    config = write_config(codex_home, OPEN + OWNED_HOOKS + '\n' + FOREIGN + CLOSE + tail)

    run_script(codex_home)

    assert config.read_text() == OPEN + OWNED_HOOKS + CLOSE + '\n' + FOREIGN + tail


def test_blank_lines_before_the_closing_marker_are_dropped(codex_home: Path):
    config = write_config(codex_home, OPEN + OWNED_HOOKS + '\n\n' + CLOSE)

    run_script(codex_home)

    assert config.read_text() == OPEN + OWNED_HOOKS + CLOSE


@pytest.mark.parametrize(
    'content',
    [
        pytest.param(HEADER + MCP_BLOCK + OPEN + OWNED_HOOKS + CLOSE, id='clean-block'),
        pytest.param(HEADER + MCP_BLOCK, id='no-hooks-block'),
        pytest.param(OPEN + OWNED_HOOKS + '\n' + FOREIGN, id='unterminated-block'),
    ],
)
def test_config_without_a_repairable_block_is_left_untouched(codex_home: Path, content: str):
    config = write_config(codex_home, content)
    before = config.stat().st_mtime_ns

    run_script(codex_home)

    assert config.read_text() == content
    assert config.stat().st_mtime_ns == before


def test_missing_config_is_not_created(codex_home: Path):
    run_script(codex_home)

    assert not (codex_home / 'config.toml').exists()
