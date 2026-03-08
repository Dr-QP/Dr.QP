#!/usr/bin/env python3

"""Tests for delegated skill validation and retained local checks."""

from __future__ import annotations

from pathlib import Path

from validate_agents.main import main


def _write_skill(skill_dir: Path, name: str, body: str) -> None:
    skill_dir.mkdir(parents=True, exist_ok=True)
    (skill_dir / 'SKILL.md').write_text(
        f"""---
name: {name}
description: A comprehensive description of what this skill does and when to use it.
---
{body}
"""
    )


def test_issue310_skill_spec_validation_delegates_to_skills_ref(
    tmp_path: Path, monkeypatch, capsys
) -> None:
    """Spec-level skill checks should come from skills-ref."""
    _write_skill(tmp_path / 'valid-skill', 'valid-skill', '# Valid Skill\n')

    calls: list[Path] = []

    def fake_validate(skill_dir: Path) -> list[str]:
        calls.append(skill_dir)
        return ['sentinel skills-ref failure']

    monkeypatch.setattr('validate_agents.core.skills_ref_validate', fake_validate)

    exit_code = main([str(tmp_path)])
    captured = capsys.readouterr()

    assert exit_code == 1
    assert calls == [tmp_path / 'valid-skill']
    assert 'sentinel skills-ref failure' in captured.out


def test_issue310_skill_validation_keeps_duplicate_name_check(
    tmp_path: Path, monkeypatch, capsys
) -> None:
    """Duplicate skill names should still fail locally."""
    _write_skill(tmp_path / 'skill-one', 'duplicate-skill', '# Skill One\n')
    _write_skill(tmp_path / 'skill-two', 'duplicate-skill', '# Skill Two\n')

    monkeypatch.setattr(
        'validate_agents.core.skills_ref_validate',
        lambda skill_dir: [],
    )

    exit_code = main([str(tmp_path)])
    captured = capsys.readouterr()

    assert exit_code == 1
    assert 'duplicate' in captured.out.lower()
    assert 'duplicate-skill' in captured.out


def test_issue310_skill_validation_keeps_broken_link_check(
    tmp_path: Path, monkeypatch, capsys
) -> None:
    """Broken skill links should still fail locally."""
    _write_skill(
        tmp_path / 'broken-link-skill',
        'broken-link-skill',
        '# Broken Link Skill\n\nSee [missing](missing.md).\n',
    )

    monkeypatch.setattr(
        'validate_agents.core.skills_ref_validate',
        lambda skill_dir: [],
    )

    exit_code = main([str(tmp_path)])
    captured = capsys.readouterr()

    assert exit_code == 1
    assert 'missing.md' in captured.out
