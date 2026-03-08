#!/usr/bin/env python3

"""Tests for delegated skill validation and retained local checks."""

from __future__ import annotations

from pathlib import Path

import pytest

from validate_agent_files.main import main
from validate_agent_files.validators.skill import (
    SkillFrontmatterValidator,
    SkillStructureValidator,
)


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

    monkeypatch.setattr('validate_agent_files.core.skills_ref_validate', fake_validate)

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
        'validate_agent_files.core.skills_ref_validate',
        lambda skill_dir: [],
    )

    exit_code = main([str(tmp_path)])
    captured = capsys.readouterr()

    assert exit_code == 1
    assert 'duplicate' in captured.out.lower()
    assert 'duplicate-skill' in captured.out


def test_issue310_skills_ref_is_primary_for_skill_validation(
    tmp_path: Path, monkeypatch, capsys
) -> None:
    """A failing skills-ref result should stop local skill-specific checks for that file."""
    _write_skill(
        tmp_path / 'broken-skill',
        'broken-skill',
        '# Broken Skill\n\nSee [missing](missing.md).\n',
    )

    monkeypatch.setattr(
        'validate_agent_files.core.skills_ref_validate',
        lambda skill_dir: ['sentinel skills-ref failure'],
    )

    exit_code = main([str(tmp_path)])
    captured = capsys.readouterr()

    assert exit_code == 1
    assert 'sentinel skills-ref failure' in captured.out
    assert 'missing.md' not in captured.out


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
        'validate_agent_files.core.skills_ref_validate',
        lambda skill_dir: [],
    )

    exit_code = main([str(tmp_path)])
    captured = capsys.readouterr()

    assert exit_code == 1
    assert 'missing.md' in captured.out


@pytest.mark.parametrize(
    ('frontmatter', 'message'),
    [
        ({'description': 'Valid enough description'}, "Missing required field: 'name'"),
        (
            {'name': 12, 'description': 'Valid enough description'},
            'Name must be a string, got: int',
        ),
        (
            {'name': 'Not Valid', 'description': 'Valid enough description'},
            "Name must be lowercase with hyphens (e.g., 'my-skill'), got: 'Not Valid'",
        ),
        (
            {'name': 'a' * 65, 'description': 'Valid enough description'},
            'Name exceeds maximum length of 64 characters',
        ),
        ({'name': 'valid-skill'}, "Missing required field: 'description'"),
        ({'name': 'valid-skill', 'description': 12}, 'Description must be a string, got: int'),
        (
            {'name': 'valid-skill', 'description': 'too short'},
            'Description must be at least 10 characters',
        ),
        (
            {'name': 'valid-skill', 'description': 'a' * 1025},
            'Description exceeds maximum length of 1024 characters',
        ),
    ],
)
def test_issue310_skill_frontmatter_validator_rejects_invalid_name_and_description_cases(
    frontmatter: dict,
    message: str,
) -> None:
    """Skill frontmatter validation should cover all error branches."""
    issues = SkillFrontmatterValidator().validate(frontmatter)

    assert [issue.section for issue in issues] == ['frontmatter']
    assert issues[0].message == message


def test_issue310_skill_frontmatter_validator_warns_on_vague_description_terms() -> None:
    """Warning mode should flag vague terminology in descriptions."""
    issues = SkillFrontmatterValidator().validate(
        {
            'name': 'valid-skill',
            'description': 'This skill bundles helpers, tools, and various adapters.',
        },
        show_warnings=True,
    )

    assert len(issues) == 1
    assert issues[0].level.value == 'warning'
    assert 'various' in issues[0].message


@pytest.mark.parametrize(
    ('body', 'show_warnings', 'expected_level', 'expected_message'),
    [
        ('', False, 'error', 'Body content is empty'),
        (
            'Paragraph only\n',
            False,
            'error',
            "Missing required top-level header (e.g., '# Title')",
        ),
        (
            '# Title\n\nshort\n',
            True,
            'warning',
            "Top-level section '# Title' appears to be empty or very short",
        ),
    ],
)
def test_issue310_skill_structure_validator_reports_problem_cases(
    body: str,
    show_warnings: bool,
    expected_level: str,
    expected_message: str,
) -> None:
    """Structure validation should cover empty, headerless, and short sections."""
    issues = SkillStructureValidator().validate(body, show_warnings=show_warnings)

    assert len(issues) == 1
    assert issues[0].level.value == expected_level
    assert issues[0].message == expected_message


def test_issue310_skill_structure_validator_accepts_sufficient_top_section() -> None:
    """A well-formed top section should not produce structure issues."""
    issues = SkillStructureValidator().validate(
        '# Title\n\nThis section contains more than enough detail to pass.\n',
        show_warnings=True,
    )

    assert issues == []
