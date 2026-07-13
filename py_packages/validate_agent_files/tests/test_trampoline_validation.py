#!/usr/bin/env python3

"""Tests for Codex trampoline sync validation (spec 01, AC10)."""

from __future__ import annotations

from pathlib import Path

from validate_agent_files.main import main


def _write_agent(claude_agents: Path, stem: str, name: str, description: str) -> None:
    (claude_agents / f'{stem}.agent.md').write_text(
        f"""---
name: {name}
description: {description}
tools: [read]
---

# {name}
"""
    )


def _write_trampoline(codex_agents: Path, stem: str, name: str, description: str) -> None:
    (codex_agents / f'{stem}.md').write_text(
        f"""---
name: {name}
description: {description}
---

Read and follow `.claude/agents/{stem}.agent.md`.
"""
    )


def _make_catalog(tmp_path: Path) -> tuple[Path, Path]:
    claude_agents = tmp_path / '.claude' / 'agents'
    codex_agents = tmp_path / '.codex' / 'agents'
    claude_agents.mkdir(parents=True)
    codex_agents.mkdir(parents=True)
    return claude_agents, codex_agents


def test_matching_trampoline_pair_passes(tmp_path: Path, capsys) -> None:
    """A canonical agent with a byte-equal trampoline validates cleanly."""
    claude_agents, codex_agents = _make_catalog(tmp_path)
    _write_agent(claude_agents, 'demo', 'Demo Agent', 'A demo agent for testing.')
    _write_trampoline(codex_agents, 'demo', 'Demo Agent', 'A demo agent for testing.')

    exit_code = main([str(tmp_path / '.claude'), '--kind', 'agents'])
    captured = capsys.readouterr()

    assert exit_code == 0
    assert 'does not match' not in captured.out
    assert 'Missing Codex trampoline' not in captured.out


def test_mismatched_description_fails_naming_both_files(tmp_path: Path, capsys) -> None:
    """A trampoline description drift is reported, naming both files."""
    claude_agents, codex_agents = _make_catalog(tmp_path)
    _write_agent(claude_agents, 'demo', 'Demo Agent', 'The canonical description.')
    _write_trampoline(codex_agents, 'demo', 'Demo Agent', 'A stale description.')

    exit_code = main([str(tmp_path / '.claude'), '--kind', 'agents'])
    captured = capsys.readouterr()

    assert exit_code == 1
    assert "'description' does not match" in captured.out
    assert 'demo.agent.md' in captured.out
    assert 'demo.md' in captured.out


def test_mismatched_name_fails(tmp_path: Path, capsys) -> None:
    """A trampoline name drift is reported."""
    claude_agents, codex_agents = _make_catalog(tmp_path)
    _write_agent(claude_agents, 'demo', 'Canonical Name', 'Shared description.')
    _write_trampoline(codex_agents, 'demo', 'Stale Name', 'Shared description.')

    exit_code = main([str(tmp_path / '.claude'), '--kind', 'agents'])
    captured = capsys.readouterr()

    assert exit_code == 1
    assert "'name' does not match" in captured.out


def test_missing_trampoline_fails(tmp_path: Path, capsys) -> None:
    """A canonical agent without a trampoline is an error."""
    claude_agents, _codex_agents = _make_catalog(tmp_path)
    _write_agent(claude_agents, 'demo', 'Demo Agent', 'A demo agent.')

    exit_code = main([str(tmp_path / '.claude'), '--kind', 'agents'])
    captured = capsys.readouterr()

    assert exit_code == 1
    assert 'Missing Codex trampoline' in captured.out


def test_orphan_trampoline_fails(tmp_path: Path, capsys) -> None:
    """A trampoline without a canonical counterpart is an error."""
    claude_agents, codex_agents = _make_catalog(tmp_path)
    _write_agent(claude_agents, 'demo', 'Demo Agent', 'A demo agent.')
    _write_trampoline(codex_agents, 'demo', 'Demo Agent', 'A demo agent.')
    _write_trampoline(codex_agents, 'ghost', 'Ghost Agent', 'No canonical file.')

    exit_code = main([str(tmp_path / '.claude'), '--kind', 'agents'])
    captured = capsys.readouterr()

    assert exit_code == 1
    assert 'Orphan Codex trampoline' in captured.out
    assert 'ghost' in captured.out


def test_individually_supplied_agents_share_one_trampoline_catalog(
    repo_tmp_path: Path, capsys
) -> None:
    """Multiple agent file arguments should be validated as one catalog."""
    claude_agents, codex_agents = _make_catalog(repo_tmp_path)
    _write_agent(claude_agents, 'first', 'First Agent', 'The first agent.')
    _write_agent(claude_agents, 'second', 'Second Agent', 'The second agent.')
    _write_trampoline(codex_agents, 'first', 'First Agent', 'The first agent.')
    _write_trampoline(codex_agents, 'second', 'Second Agent', 'The second agent.')

    exit_code = main(
        [
            str(claude_agents / 'first.agent.md'),
            str(claude_agents / 'second.agent.md'),
            '--kind',
            'agents',
        ]
    )
    captured = capsys.readouterr()

    assert exit_code == 0, captured.out
    assert 'Orphan Codex trampoline' not in captured.out


def test_single_agent_file_does_not_report_unselected_trampolines_as_orphans(
    repo_tmp_path: Path, capsys
) -> None:
    """Changed-file validation should ignore unrelated catalog trampolines."""
    claude_agents, codex_agents = _make_catalog(repo_tmp_path)
    _write_agent(claude_agents, 'changed', 'Changed Agent', 'The changed agent.')
    _write_agent(claude_agents, 'other', 'Other Agent', 'Another catalog agent.')
    _write_trampoline(codex_agents, 'changed', 'Changed Agent', 'The changed agent.')
    _write_trampoline(codex_agents, 'other', 'Other Agent', 'Another catalog agent.')

    exit_code = main([str(claude_agents / 'changed.agent.md'), '--kind', 'agents'])
    captured = capsys.readouterr()

    assert exit_code == 0, captured.out
    assert 'Orphan Codex trampoline' not in captured.out


def test_repo_catalog_trampolines_are_in_sync() -> None:
    """The real repo's four agent/trampoline pairs validate without drift."""
    repo_root = Path(__file__).resolve().parents[3]
    exit_code = main([str(repo_root / '.claude'), '--kind', 'agents', '--ci'])

    assert exit_code == 0
