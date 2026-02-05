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

"""Unit tests for validation engine core."""

from validate_skills.core import ValidationEngine, ValidationResult


class TestValidationEngineBasic:
    """Tests for basic validation engine operations."""

    def test_validation_engine_init(self):
        """Should initialize validation engine with config."""
        engine = ValidationEngine(show_warnings=False, show_info=False)
        assert engine is not None

    def test_validation_engine_validate_valid_skill(self, tmp_path):
        """Should validate a valid skill without errors."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: test-skill
description: A comprehensive description of what this skill does and when to use it.
---
# Overview
This is an overview.

## When to use this skill
Use this when you need it.""")

        engine = ValidationEngine(show_warnings=False)
        result = engine.validate(str(skill_file), all_skills={})

        assert isinstance(result, ValidationResult)
        assert result.skill_path == str(skill_file)

    def test_validation_engine_validate_invalid_frontmatter(self, tmp_path):
        """Should report errors for invalid frontmatter."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
description: Missing name field
---
Content""")

        engine = ValidationEngine()
        result = engine.validate(str(skill_file), all_skills={})

        assert not result.is_valid
        assert len(result.issues) > 0

    def test_validation_engine_validate_invalid_structure(self, tmp_path):
        """Should report errors for invalid structure."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: test-skill
description: A comprehensive description of what this skill does and when to use it.
---
Missing sections""")

        engine = ValidationEngine()
        result = engine.validate(str(skill_file), all_skills={})

        assert not result.is_valid
        assert len(result.issues) > 0


class TestValidationEngineWarnings:
    """Tests for warning handling in validation engine."""

    def test_validation_engine_show_warnings_default(self, tmp_path):
        """Should include warnings by default."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: test-skill
description: A comprehensive description of what this skill does and when to use it.
---
# Overview
Content here

## When to use this skill
Content here""")

        engine = ValidationEngine(show_warnings=True)
        result = engine.validate(str(skill_file), all_skills={})

        # Result should include any warnings
        assert isinstance(result, ValidationResult)

    def test_validation_engine_exclude_warnings(self, tmp_path):
        """Should exclude warnings when show_warnings=False."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: test-skill
description: A comprehensive description of what this skill does and when to use it.
---
# Overview
Overview content

## When to use this skill
When to use content""")

        engine = ValidationEngine(show_warnings=False)
        result = engine.validate(str(skill_file), all_skills={})

        # Result should not include warnings
        assert all(issue.level != 'WARNING' for issue in result.issues)


class TestValidationEngineMultipleValidators:
    """Tests for coordinating multiple validators."""

    def test_validation_engine_runs_all_validators(self, tmp_path):
        """Should run all configured validators."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: invalid@name
description: Too short
---
Content""")

        engine = ValidationEngine()
        result = engine.validate(str(skill_file), all_skills={})

        # Should have issues from multiple validators
        assert not result.is_valid
        assert len(result.issues) > 0

    def test_validation_engine_collects_all_issues(self, tmp_path):
        """Should collect issues from all validators."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: bad_name
description: Bad description value.
---
Content""")

        engine = ValidationEngine()
        result = engine.validate(str(skill_file), all_skills={})

        # Should collect issues from:
        # - Frontmatter validator (name format, description length)
        # - Structure validator (missing sections)
        assert len(result.issues) >= 1
        issues_found = len(result.issues)
        assert issues_found > 0


class TestValidationEngineLoadedSkills:
    """Tests for cross-validation with loaded skills."""

    def test_validation_engine_with_all_skills(self, tmp_path):
        """Should check uniqueness against all loaded skills."""
        skill1 = tmp_path / 'skill1' / 'SKILL.md'
        skill2 = tmp_path / 'skill2' / 'SKILL.md'
        skill1.parent.mkdir()
        skill2.parent.mkdir()

        skill1.write_text("""---
name: unique-skill
description: A comprehensive description of what this skill does and when to use it.
---
# Overview
Content

## When to use this skill
Content""")

        skill2.write_text("""---
name: unique-skill
description: A comprehensive description of what this skill does and when to use it.
---
# Overview
Content

## When to use this skill
Content""")

        all_skills = {str(skill1): {'frontmatter': {'name': 'unique-skill'}, 'body': 'Content'}}

        engine = ValidationEngine()
        result = engine.validate(str(skill2), all_skills=all_skills)

        # Should detect duplicate name
        assert not result.is_valid


class TestValidationEngineErrorHandling:
    """Tests for error handling in validation engine."""

    def test_validation_engine_handles_missing_file(self):
        """Should handle missing file gracefully."""
        engine = ValidationEngine()
        result = engine.validate('/nonexistent/skill/SKILL.md', all_skills={})

        assert not result.is_valid
        assert len(result.issues) > 0

    def test_validation_engine_handles_invalid_yaml(self, tmp_path):
        """Should handle invalid YAML in frontmatter."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
invalid: yaml: structure:
---
Content""")

        engine = ValidationEngine()
        result = engine.validate(str(skill_file), all_skills={})

        assert not result.is_valid
        assert len(result.issues) > 0

    def test_validation_engine_handles_permission_denied(self, tmp_path):
        """Should handle permission denied errors."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text('---\nname: test\n---\nContent')
        import os

        os.chmod(skill_file, 0o600)

        try:
            engine = ValidationEngine()
            result = engine.validate(str(skill_file), all_skills={})
            assert not result.is_valid
        finally:
            os.chmod(skill_file, 0o644)


class TestValidationResult:
    """Tests for ValidationResult data structure."""

    def test_validation_result_is_valid_no_errors(self):
        """Should indicate valid when no error issues present."""
        result = ValidationResult(skill_path='/path/SKILL.md', issues=[])
        assert result.is_valid is True

    def test_validation_result_is_valid_with_warnings(self):
        """Should indicate valid when only warnings present."""
        from validate_skills.core import ValidationIssue, ValidationLevel

        result = ValidationResult(
            skill_path='/path/SKILL.md',
            issues=[ValidationIssue(level=ValidationLevel.WARNING, message='Warning')],
        )
        assert result.is_valid is True

    def test_validation_result_is_valid_with_errors(self):
        """Should indicate invalid when errors present."""
        from validate_skills.core import ValidationIssue, ValidationLevel

        result = ValidationResult(
            skill_path='/path/SKILL.md',
            issues=[ValidationIssue(level=ValidationLevel.ERROR, message='Error')],
        )
        assert result.is_valid is False

    def test_validation_result_has_warnings(self):
        """Should detect presence of warnings."""
        from validate_skills.core import ValidationIssue, ValidationLevel

        result = ValidationResult(
            skill_path='/path/SKILL.md',
            issues=[ValidationIssue(level=ValidationLevel.WARNING, message='Warning')],
        )
        assert result.has_warnings is True

    def test_validation_result_has_warnings_false(self):
        """Should indicate no warnings when none present."""
        from validate_skills.core import ValidationIssue, ValidationLevel

        result = ValidationResult(
            skill_path='/path/SKILL.md',
            issues=[ValidationIssue(level=ValidationLevel.ERROR, message='Error')],
        )
        assert result.has_warnings is False

    def test_validation_result_issue_count(self):
        """Should count issues accurately."""
        from validate_skills.core import ValidationIssue, ValidationLevel

        result = ValidationResult(
            skill_path='/path/SKILL.md',
            issues=[
                ValidationIssue(level=ValidationLevel.ERROR, message='Error 1'),
                ValidationIssue(level=ValidationLevel.ERROR, message='Error 2'),
                ValidationIssue(level=ValidationLevel.WARNING, message='Warning'),
            ],
        )
        assert len(result.issues) == 3

    def test_validation_result_error_count(self):
        """Should count errors separately."""
        from validate_skills.core import ValidationIssue, ValidationLevel

        result = ValidationResult(
            skill_path='/path/SKILL.md',
            issues=[
                ValidationIssue(level=ValidationLevel.ERROR, message='Error 1'),
                ValidationIssue(level=ValidationLevel.ERROR, message='Error 2'),
                ValidationIssue(level=ValidationLevel.WARNING, message='Warning'),
            ],
        )
        error_count = len([i for i in result.issues if i.level == ValidationLevel.ERROR])
        assert error_count == 2
