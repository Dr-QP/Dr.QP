"""Tests for scripts/renovate-post-upgrade.sh, with its tools replaced by recording stubs."""

from __future__ import annotations

from collections.abc import Iterator
import os
from pathlib import Path
import shutil
import subprocess
from uuid import uuid4

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = 'scripts/renovate-post-upgrade.sh'
TMP_ROOT = REPO_ROOT / '.tmp'
LOCK = '.devcontainer/devcontainer-lock.json'

# Each stub logs its arguments; STUB_* variables choose its outcome per call.
UV_STUB = """#!/usr/bin/env bash
set -euo pipefail
printf 'uv %s\\n' "$*" >> "$STUB_LOG"
printf '%s\n' "${SKIP-<unset>}" >> "$STUB_LOG.skip"
count_file="$STUB_LOG.precommit"
count=$(( $(cat "$count_file" 2>/dev/null || echo 0) + 1 ))
echo "$count" > "$count_file"
if [[ $count -eq 1 && -n "${STUB_PRECOMMIT_REWRITE:-}" ]]; then
  echo formatted > "$STUB_PRECOMMIT_REWRITE"
  exit 1
fi
exit "${STUB_PRECOMMIT_EXIT:-0}"
"""
BUNX_STUB = """#!/usr/bin/env bash
set -euo pipefail
printf 'bunx %s\\n' "$*" >> "$STUB_LOG"
echo '{"regenerated": true}' > .devcontainer/devcontainer-lock.json
"""


@pytest.fixture
def repo() -> Iterator[Path]:
    """Create a Git repository holding the script, with one committed file per concern."""
    TMP_ROOT.mkdir(exist_ok=True)
    root = TMP_ROOT / f'post-upgrade-{uuid4().hex}'
    (root / 'scripts').mkdir(parents=True)
    (root / '.devcontainer').mkdir()
    (root / 'bin').mkdir()
    shutil.copy(REPO_ROOT / SCRIPT, root / SCRIPT)
    for name, body in (('uv', UV_STUB), ('bunx', BUNX_STUB)):
        (root / 'bin' / name).write_text(body)
        (root / 'bin' / name).chmod(0o755)
    (root / '.gitignore').write_text('bin/\nstub.log*\n')
    for path in ('.devcontainer/devcontainer.json', LOCK, 'pins.yml', 'gone.yml'):
        (root / path).write_text('original\n')
    try:
        git(root, 'init', '--quiet', '--initial-branch=main')
        git(root, 'add', '-A')
        git(root, 'commit', '--quiet', '-m', 'fixture')
        yield root
    finally:
        shutil.rmtree(root)


def git(root: Path, *arguments: str) -> None:
    """Run Git in the fixture repository, isolated from any outer Git environment."""
    identity = ['-c', 'user.name=Fixture', '-c', 'user.email=fixture@example.invalid']
    env = {k: v for k, v in os.environ.items() if not k.startswith('GIT_')}
    subprocess.run(['git', *identity, *arguments], cwd=root, check=True, env=env)


def run(root: Path, **stub_env: str) -> tuple[int, list[str]]:
    """Run the script with stubbed tools; return its exit code and the stub call log."""
    log = root / 'stub.log'
    env = {k: v for k, v in os.environ.items() if not k.startswith('GIT_')}
    env.update(stub_env, STUB_LOG=str(log), PATH=f'{root / "bin"}:{env["PATH"]}')
    result = subprocess.run([str(root / SCRIPT)], cwd=root, env=env, check=False)
    calls = log.read_text().splitlines() if log.exists() else []
    return result.returncode, calls


def test_changed_files_reach_pre_commit(repo: Path) -> None:
    """Modified and new files are checked; deleted files are skipped."""
    (repo / 'pins.yml').write_text('bumped\n')
    (repo / 'new.yml').write_text('added\n')
    (repo / 'gone.yml').unlink()

    code, calls = run(repo)

    assert code == 0
    assert calls == ['uv run --frozen pre-commit run --files pins.yml new.yml']


def test_devcontainer_bump_regenerates_the_lock_before_pre_commit(repo: Path) -> None:
    """A changed devcontainer.json runs the pinned CLI, and pre-commit sees the new lock."""
    (repo / '.devcontainer/devcontainer.json').write_text('bumped\n')

    code, calls = run(repo)

    assert code == 0
    assert calls[0].startswith('bunx --package @devcontainers/cli@')
    assert calls[0].endswith(' devcontainer upgrade --workspace-folder .')
    assert (
        calls[1]
        == f'uv run --frozen pre-commit run --files {LOCK} .devcontainer/devcontainer.json'
    )


def test_no_devcontainer_change_leaves_the_lock_alone(repo: Path) -> None:
    """The devcontainer CLI runs only when devcontainer.json changed."""
    (repo / 'pins.yml').write_text('bumped\n')

    _, calls = run(repo)

    assert not any(call.startswith('bunx') for call in calls)


def test_a_hook_rewrite_passes_on_the_second_pass(repo: Path) -> None:
    """A first pass that only rewrites files is followed by a clean second pass."""
    (repo / 'pins.yml').write_text('bumped\n')

    code, calls = run(repo, STUB_PRECOMMIT_REWRITE=str(repo / 'pins.yml'))

    assert code == 0
    assert [call for call in calls if 'pre-commit' in call] == [
        'uv run --frozen pre-commit run --files pins.yml',
        'uv run --frozen pre-commit run --files pins.yml',
    ]
    assert (repo / 'pins.yml').read_text() == 'formatted\n'


def test_a_hook_still_failing_on_the_second_pass_fails(repo: Path) -> None:
    """A hook that fails both passes fails the task."""
    (repo / 'pins.yml').write_text('bumped\n')

    code, _ = run(repo, STUB_PRECOMMIT_EXIT='1')

    assert code != 0


def test_nothing_changed_runs_nothing(repo: Path) -> None:
    """With no edits against HEAD the task is a no-op."""
    code, calls = run(repo)

    assert code == 0
    assert calls == []


def pre_commit_skips(root: Path) -> list[str]:
    """Return the SKIP value each pre-commit invocation saw."""
    return (root / 'stub.log.skip').read_text().splitlines()


def test_pre_commit_skips_the_default_branch_guard_on_every_pass(repo: Path) -> None:
    """Both passes run on the base branch, so both skip no-commit-to-branch."""
    (repo / 'pins.yml').write_text('bumped\n')

    code, _ = run(repo, STUB_PRECOMMIT_REWRITE=str(repo / 'pins.yml'))

    assert code == 0
    assert pre_commit_skips(repo) == ['no-commit-to-branch', 'no-commit-to-branch']


def test_pre_commit_skip_keeps_the_callers_value(repo: Path) -> None:
    """A SKIP set by the caller is extended, not replaced."""
    (repo / 'pins.yml').write_text('bumped\n')

    code, _ = run(repo, SKIP='fixture-hook')

    assert code == 0
    assert pre_commit_skips(repo) == ['fixture-hook,no-commit-to-branch']
