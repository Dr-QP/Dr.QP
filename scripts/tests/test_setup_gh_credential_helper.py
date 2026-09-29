from pathlib import Path
import shutil
import subprocess

import pytest

REPO_ROOT = Path(__file__).parents[2]
SCRIPT = REPO_ROOT / '.devcontainer/scripts/setup-gh-credential-helper.sh'
POST_START = REPO_ROOT / '.devcontainer/scripts/postStartCommand.sh'
GH_HELPER = '!gh auth git-credential'


def run_setup(
    tmp_path: Path,
    *,
    gh_installed: bool = True,
    authenticated: bool = True,
    helper: str | None = None,
    keyring_env: str | None = None,
) -> tuple[subprocess.CompletedProcess[str], str]:
    script = tmp_path / 'workspace/.devcontainer/scripts' / SCRIPT.name
    script.parent.mkdir(parents=True)
    shutil.copy2(SCRIPT, script)
    if keyring_env is not None:
        (tmp_path / 'workspace/.tmp').mkdir()
        (tmp_path / 'workspace/.tmp/keyring-session.env').write_text(keyring_env)
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    for tool in ('git', 'bash', 'env', 'dirname'):
        (fake_bin / tool).symlink_to(shutil.which(tool))
    if gh_installed:
        (fake_bin / 'gh').write_text(
            '#!/bin/sh\n'
            'if [ "$1 $2" = "auth status" ]; then [ "$GH_TEST_AUTHENTICATED" = 1 ]; exit; fi\n'
            'if [ "$1 $2" = "auth setup-git" ]; then\n'
            f"  git config --global credential.https://github.com.helper '{GH_HELPER}'\n"
            'fi\n'
        )
        (fake_bin / 'gh').chmod(0o755)
    gitconfig = tmp_path / 'gitconfig'
    gitconfig.touch()
    env = {
        'PATH': str(fake_bin),
        'HOME': str(tmp_path),
        'GIT_CONFIG_GLOBAL': str(gitconfig),
        'GIT_CONFIG_NOSYSTEM': '1',
        'GH_TEST_AUTHENTICATED': '1' if authenticated else '0',
    }
    if helper is not None:
        subprocess.run(
            ['git', 'config', '--global', 'credential.helper', helper], env=env, check=True
        )
    result = subprocess.run([script], env=env, text=True, capture_output=True)
    configured = subprocess.run(
        ['git', 'config', '--get-urlmatch', 'credential.helper', 'https://github.com'],
        env=env,
        text=True,
        capture_output=True,
    ).stdout.strip()
    return result, configured


def test_configures_gh_helper_when_authenticated_and_unset(tmp_path):
    result, configured = run_setup(tmp_path)
    assert result.returncode == 0, result.stderr
    assert configured == GH_HELPER


@pytest.mark.parametrize(('gh_installed', 'authenticated'), [(False, False), (True, False)])
def test_skips_without_authenticated_gh(tmp_path, gh_installed, authenticated):
    result, configured = run_setup(
        tmp_path, gh_installed=gh_installed, authenticated=authenticated
    )
    assert result.returncode == 0, result.stderr
    assert configured == ''


def test_keeps_existing_helper(tmp_path):
    result, configured = run_setup(tmp_path, helper='store')
    assert result.returncode == 0, result.stderr
    assert configured == 'store'


def test_reads_gh_login_through_saved_keyring_session(tmp_path):
    result, configured = run_setup(
        tmp_path, authenticated=False, keyring_env='export GH_TEST_AUTHENTICATED=1\n'
    )
    assert result.returncode == 0, result.stderr
    assert configured == GH_HELPER


def test_post_start_runs_setup_after_keyring():
    post_start = POST_START.read_text()
    helper = post_start.index('"$script_dir/setup-gh-credential-helper.sh"')
    assert post_start.index('"$script_dir/setup-keyring.sh"') < helper
