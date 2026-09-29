"""Tests that this repository's pre-commit configuration rejects commits on main and master."""

from __future__ import annotations

from collections.abc import Iterator
import os
from pathlib import Path
import shutil
import subprocess
from uuid import uuid4

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
TMP_ROOT = REPO_ROOT / '.tmp'
HOOK_ID = 'no-commit-to-branch'
FIXTURE_ENV = {
    'GIT_AUTHOR_NAME': 'Fixture Author',
    'GIT_AUTHOR_EMAIL': 'fixture@example.invalid',
    'GIT_COMMITTER_NAME': 'Fixture Author',
    'GIT_COMMITTER_EMAIL': 'fixture@example.invalid',
}


def git(cwd: Path, *args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    """Run a fixture git command."""
    return subprocess.run(
        ['git', *args],
        cwd=cwd,
        check=check,
        capture_output=True,
        text=True,
        env={**os.environ, **FIXTURE_ENV},
    )


def guard_only_config() -> dict[str, object]:
    """Return the repository's pre-commit config reduced to the default-branch guard hook."""
    config = yaml.safe_load((REPO_ROOT / '.pre-commit-config.yaml').read_text())
    for repo in config['repos']:
        hooks = [hook for hook in repo.get('hooks', []) if hook['id'] == HOOK_ID]
        if hooks:
            return {'repos': [{**repo, 'hooks': hooks}]}
    pytest.fail(f'{HOOK_ID} is not configured in .pre-commit-config.yaml')


@pytest.fixture
def repo() -> Iterator[Path]:
    """Create a Git repository with only the guard hook installed."""
    TMP_ROOT.mkdir(exist_ok=True)
    root = TMP_ROOT / f'pre-commit-guard-{uuid4().hex}'
    root.mkdir()
    try:
        git(root, 'init', '--quiet', '--initial-branch=main')
        (root / '.pre-commit-config.yaml').write_text(yaml.safe_dump(guard_only_config()))
        subprocess.run(['pre-commit', 'install'], cwd=root, check=True, capture_output=True)
        git(root, 'add', '-A')
        git(root, 'commit', '--quiet', '--no-verify', '-m', 'fixture')
        yield root
    finally:
        shutil.rmtree(root)


def commit_change(repo: Path) -> subprocess.CompletedProcess[str]:
    """Stage a new file and commit it through the installed hooks."""
    (repo / f'{uuid4().hex}.txt').write_text('change\n')
    git(repo, 'add', '-A')
    return git(repo, 'commit', '-m', 'change', check=False)


@pytest.mark.parametrize('branch', ['main', 'master'])
def test_commit_on_default_branch_is_refused(repo: Path, branch: str) -> None:
    """A direct git commit on main or master fails the hook and creates no commit."""
    # Arrange
    git(repo, 'switch', '--quiet', '-C', branch)
    head = git(repo, 'rev-parse', 'HEAD').stdout

    # Act
    completed = commit_change(repo)

    # Assert
    assert completed.returncode != 0
    assert HOOK_ID in completed.stdout + completed.stderr
    assert git(repo, 'rev-parse', 'HEAD').stdout == head


def test_commit_on_feature_branch_succeeds(repo: Path) -> None:
    """The guard lets a commit on a feature branch through."""
    # Arrange
    git(repo, 'switch', '--quiet', '-c', 'feature')
    head = git(repo, 'rev-parse', 'HEAD').stdout

    # Act
    completed = commit_change(repo)

    # Assert
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert git(repo, 'rev-parse', 'HEAD').stdout != head


def test_pre_commit_run_on_default_branch_passes_when_the_guard_is_skipped(repo: Path) -> None:
    """Tooling that runs hooks on main, not commits, can skip the guard by id."""
    # Arrange
    (repo / 'fixture.txt').write_text('change\n')
    env = {**os.environ, **FIXTURE_ENV, 'SKIP': HOOK_ID}

    # Act
    completed = subprocess.run(
        ['pre-commit', 'run', '--files', 'fixture.txt'],
        cwd=repo,
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )

    # Assert
    assert completed.returncode == 0, completed.stdout + completed.stderr
