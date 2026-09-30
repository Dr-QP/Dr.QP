import hashlib
from pathlib import Path
import shutil
import subprocess

import pytest

REPO_ROOT = Path(__file__).parents[2]
SEED_KEY_SCRIPT = REPO_ROOT / '.devcontainer/scripts/workspace-seed-key.sh'
WORKSPACE = '/Users/dev/src/agent devcontainer'
EXPECTED_KEY = hashlib.sha256(WORKSPACE.encode()).hexdigest()[:16]


def restricted_path(tmp_path: Path, tools: list[str]) -> str:
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    for tool in ['bash', *tools]:
        found = shutil.which(tool)
        if found is None:
            pytest.skip(f'{tool} is not installed')
        (bin_dir / tool).symlink_to(found)
    return str(bin_dir)


def derive_key(path_env: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [SEED_KEY_SCRIPT, WORKSPACE],
        env={'PATH': path_env},
        text=True,
        capture_output=True,
    )


@pytest.mark.parametrize('hasher', ['sha256sum', 'shasum'])
def test_seed_key_is_sha256_prefix_with_either_hasher(tmp_path, hasher):
    result = derive_key(restricted_path(tmp_path, [hasher]))

    assert result.returncode == 0, result.stderr
    assert result.stdout == f'{EXPECTED_KEY}\n'


def test_seed_key_fails_without_any_hasher(tmp_path):
    result = derive_key(restricted_path(tmp_path, []))

    assert result.returncode != 0
    assert result.stdout == ''
