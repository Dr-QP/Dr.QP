#!/usr/bin/env python3
# Copyright (c) 2026 Dr.QP
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Tests for skill validation logic."""

import sys
from pathlib import Path

import pytest

# Import dependencies
sys.path.insert(0, str(Path(__file__).parent.parent.parent))
from skill_validators import ValidationIssue, ValidationLevel, ValidationResult

# Import module under test
validate_skills = __import__('validate-skills')


class TestPydanticErrorsToIssues:
    """Test suite for pydantic_errors_to_issues function."""

    def test_converts_single_error(self):
        """Should convert a single Pydantic error to ValidationIssue."""
        errors = [{'loc': ('name',), 'msg': 'field required'}]
        result = validate_skills.pydantic_errors_to_issues(errors, 'frontmatter')

        assert len(result) == 1
        assert isinstance(result[0], ValidationIssue)
        assert result[0].level == ValidationLevel.ERROR
        assert 'name' in result[0].message
        assert 'field required' in result[0].message
        assert result[0].section == 'frontmatter'

    def test_converts_multiple_errors(self):
        """Should convert multiple Pydantic errors."""
        errors = [
            {'loc': ('name',), 'msg': 'field required'},
            {'loc': ('description',), 'msg': 'field required'},
            {'loc': ('license',), 'msg': 'field required'},
        ]
        result = validate_skills.pydantic_errors_to_issues(errors, 'frontmatter')

        assert len(result) == 3
        assert all(isinstance(issue, ValidationIssue) for issue in result)

    def test_handles_nested_field_location(self):
        """Should format nested field locations correctly."""
        errors = [{'loc': ('config', 'option', 'value'), 'msg': 'invalid value'}]
        result = validate_skills.pydantic_errors_to_issues(errors, 'structure')

        assert len(result) == 1
        assert 'config.option.value' in result[0].message

    def test_handles_empty_location(self):
        """Should handle errors with empty location tuple."""
        errors = [{'loc': (), 'msg': 'validation error'}]
        result = validate_skills.pydantic_errors_to_issues(errors, 'body')

        assert len(result) == 1
        assert result[0].message == 'validation error'

    def test_assigns_correct_section(self):
        """Should assign the provided section to all issues."""
        errors = [
            {'loc': ('field1',), 'msg': 'error 1'},
            {'loc': ('field2',), 'msg': 'error 2'},
        ]
        result = validate_skills.pydantic_errors_to_issues(errors, 'custom_section')

        assert all(issue.section == 'custom_section' for issue in result)

    def test_returns_empty_list_for_no_errors(self):
        """Should return empty list when no errors provided."""
        result = validate_skills.pydantic_errors_to_issues([], 'frontmatter')
        assert result == []


class TestValidateSkill:
    """Test suite for validate_skill function."""

    def test_validates_good_skill_successfully(self, good_skill_path: Path):
        """Should validate a properly structured skill without errors."""
        all_skills = validate_skills.load_all_skills([str(good_skill_path)])
        repo_root = str(good_skill_path.parent.parent.parent.parent)

        result = validate_skills.validate_skill(
            str(good_skill_path), show_warnings=False, all_skills=all_skills, repo_root=repo_root
        )

        assert isinstance(result, ValidationResult)
        assert result.skill_path == str(good_skill_path)

    def test_detects_frontmatter_validation_errors(self, tmp_path: Path):
        """Should detect and report frontmatter validation errors."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: Bad_Name_With_Underscores
description: Too short
---
# Skill
""")

        all_skills = validate_skills.load_all_skills([str(skill_file)])
        result = validate_skills.validate_skill(
            str(skill_file), show_warnings=False, all_skills=all_skills, repo_root=str(tmp_path)
        )

        assert len(result.issues) > 0
        assert any(issue.section == 'frontmatter' for issue in result.issues)

    def test_detects_structure_validation_errors(self, tmp_path: Path):
        """Should detect missing required sections in body."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: test-skill
description: A valid description for testing purposes
license: MIT
---
# Test Skill

This skill is missing required sections.
""")

        all_skills = validate_skills.load_all_skills([str(skill_file)])
        result = validate_skills.validate_skill(
            str(skill_file), show_warnings=False, all_skills=all_skills, repo_root=str(tmp_path)
        )

        assert len(result.issues) > 0
        assert any(issue.section == 'structure' for issue in result.issues)

    def test_includes_warnings_when_requested(self, good_skill_path: Path):
        """Should include warnings when show_warnings=True."""
        all_skills = validate_skills.load_all_skills([str(good_skill_path)])
        repo_root = str(good_skill_path.parent.parent.parent.parent)

        result = validate_skills.validate_skill(
            str(good_skill_path), show_warnings=True, all_skills=all_skills, repo_root=repo_root
        )

        _ = [issue for issue in result.issues if issue.level == ValidationLevel.WARNING]

    def test_handles_file_read_error(self, tmp_path: Path, monkeypatch):
        """Should handle file read errors gracefully."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text('---\nname: test\n---\n# Test')

        original_open = open

        def mock_open_error(*args, **kwargs):
            if 'SKILL.md' in str(args[0]):
                raise OSError('Permission denied')
            return original_open(*args, **kwargs)

        monkeypatch.setattr('builtins.open', mock_open_error)

        result = validate_skills.validate_skill(
            str(skill_file), show_warnings=False, all_skills={}, repo_root=str(tmp_path)
        )

        assert len(result.issues) > 0
        assert any('Error validating skill' in issue.message for issue in result.issues)

    def test_handles_missing_frontmatter(self, tmp_path: Path):
        """Should handle skill files with missing frontmatter."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text('# Skill Without Frontmatter\n\nContent here.')

        all_skills = validate_skills.load_all_skills([str(skill_file)])
        result = validate_skills.validate_skill(
            str(skill_file), show_warnings=False, all_skills=all_skills, repo_root=str(tmp_path)
        )

        assert len(result.issues) > 0

    def test_runs_custom_validators(self, good_skill_path: Path):
        """Should run UniquenessValidator and CrossReferenceValidator."""
        all_skills = validate_skills.load_all_skills([str(good_skill_path)])
        repo_root = str(good_skill_path.parent.parent.parent.parent)

        result = validate_skills.validate_skill(
            str(good_skill_path), show_warnings=True, all_skills=all_skills, repo_root=repo_root
        )

        assert isinstance(result, ValidationResult)

    def test_validates_with_empty_all_skills_dict(self, good_skill_path: Path):
        """Should handle validation with empty all_skills dictionary."""
        result = validate_skills.validate_skill(
            str(good_skill_path), show_warnings=False, all_skills={}, repo_root='/'
        )

        assert isinstance(result, ValidationResult)
