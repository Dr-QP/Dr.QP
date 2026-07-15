#!/usr/bin/env python3

"""Regression tests for the Super-Linter tool-version validator."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess


def _copy_validator_workspace(repo_path: Path) -> Path:
    """Copy validator inputs into an isolated workspace."""
    workspace_root = Path(__file__).resolve().parents[3]
    validator_paths = (
        'scripts/validate-super-linter-tool-versions.sh',
        'scripts/super-linter-defaults.sh',
        '.pre-commit-config.yaml',
        'docker/ros/ansible/playbooks/roles/dev_tools/defaults/main.yml',
        '.github/workflows/reformat.yml',
    )
    for relative_path in validator_paths:
        source_path = workspace_root / relative_path
        destination_path = repo_path / relative_path
        destination_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_path, destination_path)

    return repo_path / 'scripts' / 'validate-super-linter-tool-versions.sh'


def _write_fake_docker(fake_bin: Path) -> None:
    """Write a Docker replacement that reports the pinned tool versions."""
    fake_docker = fake_bin / 'docker'
    fake_docker.write_text(
        """#!/usr/bin/env bash
printf '%s\\n' \\
  prettier=3.8.1 \\
  clang-format=21.1.2 \\
  ansible-lint=26.1.1 \\
  hadolint=2.14.0 \\
  ruff=0.15.0 \\
  shellcheck=0.11.0 \\
  gitleaks=8.30.0 \\
  actionlint=1.7.10 \\
  zizmor=1.22.0
"""
    )
    fake_docker.chmod(0o755)


def test_validator_reports_all_mismatches_before_failing(repo_tmp_path: Path) -> None:
    """Every mismatch should be reported even when the first check fails."""
    validator = _copy_validator_workspace(repo_tmp_path)
    pre_commit_config = repo_tmp_path / '.pre-commit-config.yaml'
    pre_commit_config.write_text(
        pre_commit_config.read_text()
        .replace('prettier@3.8.1', 'prettier@9.9.9')
        .replace('rev: v21.1.2', 'rev: v21.1.1')
        .replace('hadolint:v2.14.0 hadolint', 'hadolint:v2.13.0 hadolint')
    )
    reformat_workflow = repo_tmp_path / '.github/workflows/reformat.yml'
    reformat_workflow.write_text(
        reformat_workflow.read_text().replace(
            'super-linter/super-linter@v8.5.0',
            'super-linter/super-linter@v8.4.0',
        )
    )

    fake_bin = repo_tmp_path / 'fake-bin'
    fake_bin.mkdir()
    _write_fake_docker(fake_bin)
    environment = os.environ.copy()
    environment['PATH'] = f'{fake_bin}:{environment["PATH"]}'

    completed = subprocess.run(
        [str(validator)],
        cwd=repo_tmp_path,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode != 0
    assert 'Version mismatch for prettier' in completed.stderr
    assert 'Version mismatch for clang-format' in completed.stderr
    assert 'Version mismatch for hadolint entry' in completed.stderr
    assert 'Version mismatch for Super-Linter' in completed.stderr


def test_validator_fails_without_a_recognized_super_linter_workflow_tag(
    repo_tmp_path: Path,
) -> None:
    """A workflow image must use a recognized Super-Linter version tag."""
    validator = _copy_validator_workspace(repo_tmp_path)
    reformat_workflow = repo_tmp_path / '.github/workflows/reformat.yml'
    reformat_workflow.write_text(
        reformat_workflow.read_text().replace(
            'super-linter/super-linter@v8.5.0',
            'super-linter/super-linter@latest',
        )
    )

    fake_bin = repo_tmp_path / 'fake-bin'
    fake_bin.mkdir()
    _write_fake_docker(fake_bin)
    environment = os.environ.copy()
    environment['PATH'] = f'{fake_bin}:{environment["PATH"]}'

    completed = subprocess.run(
        [str(validator)],
        cwd=repo_tmp_path,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode != 0
    assert 'All pre-commit and local tool versions match' not in completed.stdout
