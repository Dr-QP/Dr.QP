import json
import os
from pathlib import Path
import stat
import subprocess

import pytest

SCRIPT = Path(__file__).parents[2] / '.devcontainer/scripts/configure-opencode.py'
SCHEMA = 'https://opencode.ai/config.json'
MANAGED = {
    '$schema': SCHEMA,
    'permission': {'*': 'allow', 'external_directory': {'/**': 'allow'}},
}


@pytest.fixture
def config_dir(tmp_path: Path) -> Path:
    """Return an isolated OpenCode config directory."""
    return tmp_path / 'opencode'


def run_script(config_dir: Path) -> None:
    """Run the configure script against ``config_dir``."""
    subprocess.run(
        [SCRIPT],
        env=os.environ | {'OPENCODE_CONFIG_DIR': str(config_dir)},
        check=True,
    )


def mode(path: Path) -> int:
    """Return ``path``'s permission bits."""
    return stat.S_IMODE(path.stat().st_mode)


def test_absent_config_is_created_with_policy_and_private_modes(config_dir: Path) -> None:
    run_script(config_dir)

    config = config_dir / 'opencode.json'
    assert json.loads(config.read_text()) == MANAGED
    assert mode(config_dir) == 0o700
    assert mode(config) == 0o600


def test_existing_config_preserves_unmanaged_settings(config_dir: Path) -> None:
    config_dir.mkdir()
    config = config_dir / 'opencode.json'
    config.write_text(
        json.dumps(
            {
                '$schema': 'https://example.invalid/config.json',
                'plugin': ['/catalog/.agents/plugins/agentdev/.opencode-plugin'],
                'model': 'openai/gpt-6.1-sol',
                'permission': {
                    '*': 'ask',
                    'skill': {'iwe-ship': 'deny'},
                    'external_directory': {
                        '/**': 'ask',
                        '/catalog/**': 'deny',
                    },
                },
            }
        )
    )

    run_script(config_dir)

    assert json.loads(config.read_text()) == {
        '$schema': SCHEMA,
        'plugin': ['/catalog/.agents/plugins/agentdev/.opencode-plugin'],
        'model': 'openai/gpt-6.1-sol',
        'permission': {
            '*': 'allow',
            'skill': {'iwe-ship': 'deny'},
            'external_directory': {
                '/**': 'allow',
                '/catalog/**': 'deny',
            },
        },
    }


def test_jsonc_config_is_patched_after_json_without_losing_settings(config_dir: Path) -> None:
    config_dir.mkdir()
    json_config = config_dir / 'opencode.json'
    json_config.write_text(json.dumps({'plugin': ['/catalog/bridge']}))
    jsonc_config = config_dir / 'opencode.jsonc'
    jsonc_config.write_text(
        """{
  // This file overrides opencode.json in the same directory.
  "model": "openai/gpt-6.1-sol",
  "instructions": ["https://example.com/rules.json"], /* keep URL punctuation */
  "permission": {
    "*": "ask",
    "external_directory": {
      "/catalog/**": "deny",
    },
  },
}
"""
    )
    original_json = json_config.read_bytes()

    run_script(config_dir)

    assert json_config.read_bytes() == original_json
    assert json.loads(jsonc_config.read_text()) == {
        '$schema': SCHEMA,
        'model': 'openai/gpt-6.1-sol',
        'instructions': ['https://example.com/rules.json'],
        'permission': {
            '*': 'allow',
            'external_directory': {'/**': 'allow', '/catalog/**': 'deny'},
        },
    }


def test_opencode_config_dir_takes_precedence(tmp_path: Path) -> None:
    explicit = tmp_path / 'explicit'
    xdg = tmp_path / 'xdg'
    subprocess.run(
        [SCRIPT],
        env=os.environ
        | {
            'HOME': str(tmp_path / 'home'),
            'OPENCODE_CONFIG_DIR': str(explicit),
            'XDG_CONFIG_HOME': str(xdg),
        },
        check=True,
    )

    assert (explicit / 'opencode.json').exists()
    assert not xdg.exists()


def test_xdg_config_home_is_used_when_no_override(tmp_path: Path) -> None:
    xdg = tmp_path / 'xdg'
    env = os.environ | {'HOME': str(tmp_path / 'home'), 'XDG_CONFIG_HOME': str(xdg)}
    env.pop('OPENCODE_CONFIG_DIR', None)

    subprocess.run([SCRIPT], env=env, check=True)

    assert (xdg / 'opencode/opencode.json').exists()


def test_second_run_leaves_content_unchanged(config_dir: Path) -> None:
    run_script(config_dir)
    config = config_dir / 'opencode.json'
    first = config.read_bytes()

    run_script(config_dir)

    assert config.read_bytes() == first


def test_malformed_config_fails_and_is_left_in_place(config_dir: Path) -> None:
    config_dir.mkdir()
    config = config_dir / 'opencode.json'
    config.write_text('{"permission":')

    with pytest.raises(subprocess.CalledProcessError):
        run_script(config_dir)

    assert config.read_text() == '{"permission":'
    assert [path.name for path in config_dir.iterdir()] == ['opencode.json']
