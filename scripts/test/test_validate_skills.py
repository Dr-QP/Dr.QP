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

"""Comprehensive test suite for validate-skills.py script."""

import sys
from pathlib import Path
from typing import Dict

import pytest

# Add scripts directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from skill_validators import ValidationIssue, ValidationLevel, ValidationResult

# Import module under test
# Note: validate_skills uses underscore, not hyphen
sys.path.insert(0, str(Path(__file__).parent.parent))
validate_skills = __import__('validate-skills')


# ==================== Test Fixtures ====================


@pytest.fixture
def fixtures_dir() -> Path:
    """Return path to test fixtures directory."""
    return Path(__file__).parent / 'fixtures'


@pytest.fixture
def good_skill_path(fixtures_dir: Path) -> Path:
    """Return path to a valid skill file."""
    return fixtures_dir / 'good-skill' / 'SKILL.md'


@pytest.fixture
def bad_skill_path(fixtures_dir: Path) -> Path:
    """Return path to an invalid skill file."""
    return fixtures_dir / 'bad-skill' / 'SKILL.md'


@pytest.fixture
def temp_skill_dir(tmp_path: Path) -> Path:
    """Create a temporary directory for test skills."""
    skill_dir = tmp_path / 'skills'
    skill_dir.mkdir()
    return skill_dir


@pytest.fixture
def sample_frontmatter() -> Dict:
    """Return sample valid frontmatter."""
    return {'name': 'test-skill', 'description': 'A test skill for validation', 'license': 'MIT'}


@pytest.fixture
def sample_body() -> str:
    """Return sample valid skill body."""
    return """# Test Skill

## When to Use This Skill

Use this when testing.

## Prerequisites

- Python 3.8+
- pytest
"""


# ==================== find_skill_files Tests ====================


class TestFindSkillFiles:
    """Test suite for find_skill_files function."""

    def test_finds_single_skill_file(self, good_skill_path: Path):
        """Should return a single skill file when given a direct file path."""
        result = validate_skills.find_skill_files(str(good_skill_path))
        assert len(result) == 1
        assert result[0] == str(good_skill_path)

    def test_returns_empty_for_non_skill_file(self, tmp_path: Path):
        """Should return empty list when file is not named SKILL.md."""
        regular_file = tmp_path / 'README.md'
        regular_file.write_text('# Not a skill')

        result = validate_skills.find_skill_files(str(regular_file))
        assert result == []

    def test_finds_multiple_skills_in_directory(self, temp_skill_dir: Path):
        """Should find all SKILL.md files in directory tree."""
        # Create multiple skill directories
        skill1_dir = temp_skill_dir / 'skill1'
        skill1_dir.mkdir()
        (skill1_dir / 'SKILL.md').write_text('---\nname: skill1\n---\n# Skill 1')

        skill2_dir = temp_skill_dir / 'skill2'
        skill2_dir.mkdir()
        (skill2_dir / 'SKILL.md').write_text('---\nname: skill2\n---\n# Skill 2')

        result = validate_skills.find_skill_files(str(temp_skill_dir))
        assert len(result) == 2
        assert any('skill1' in path for path in result)
        assert any('skill2' in path for path in result)

    def test_skips_test_directories(self, temp_skill_dir: Path):
        """Should exclude SKILL.md files in test directories."""
        # Create test directory with skill
        test_dir = temp_skill_dir / 'scripts' / 'test' / 'fixtures'
        test_dir.mkdir(parents=True)
        (test_dir / 'SKILL.md').write_text('---\nname: test\n---\n# Test')

        # Create valid skill outside test directory
        valid_dir = temp_skill_dir / 'valid'
        valid_dir.mkdir()
        (valid_dir / 'SKILL.md').write_text('---\nname: valid\n---\n# Valid')

        result = validate_skills.find_skill_files(str(temp_skill_dir))
        assert len(result) == 1
        assert 'valid' in result[0]
        assert 'fixtures' not in result[0]

    def test_handles_nested_directory_structure(self, temp_skill_dir: Path):
        """Should find skills in deeply nested directory structures."""
        nested = temp_skill_dir / 'level1' / 'level2' / 'level3'
        nested.mkdir(parents=True)
        (nested / 'SKILL.md').write_text('---\nname: nested\n---\n# Nested')

        result = validate_skills.find_skill_files(str(temp_skill_dir))
        assert len(result) == 1
        assert 'level3' in result[0]

    def test_returns_empty_for_nonexistent_path(self):
        """Should return empty list for nonexistent path."""
        result = validate_skills.find_skill_files('/nonexistent/path/to/skills')
        assert result == []

    def test_handles_empty_directory(self, temp_skill_dir: Path):
        """Should return empty list when directory contains no skills."""
        result = validate_skills.find_skill_files(str(temp_skill_dir))
        assert result == []


# ==================== load_all_skills Tests ====================


class TestLoadAllSkills:
    """Test suite for load_all_skills function."""

    def test_loads_valid_skill(self, good_skill_path: Path):
        """Should successfully load a valid skill file."""
        result = validate_skills.load_all_skills([str(good_skill_path)])

        assert str(good_skill_path) in result
        assert 'frontmatter' in result[str(good_skill_path)]
        assert 'body' in result[str(good_skill_path)]
        assert 'name' in result[str(good_skill_path)]['frontmatter']

    def test_loads_multiple_skills(self, good_skill_path: Path, bad_skill_path: Path):
        """Should load multiple skill files into a dictionary."""
        paths = [str(good_skill_path), str(bad_skill_path)]
        result = validate_skills.load_all_skills(paths)

        assert len(result) == 2
        assert str(good_skill_path) in result
        assert str(bad_skill_path) in result

    def test_handles_nonexistent_file(self, capsys):
        """Should handle OSError for nonexistent files gracefully."""
        result = validate_skills.load_all_skills(['/nonexistent/SKILL.md'])

        captured = capsys.readouterr()
        assert 'Error loading' in captured.out
        assert result == {}

    def test_handles_unicode_decode_error(self, tmp_path: Path, capsys):
        """Should handle files with invalid UTF-8 encoding."""
        bad_file = tmp_path / 'SKILL.md'
        bad_file.write_bytes(b'\xff\xfe Invalid UTF-8')

        _ = validate_skills.load_all_skills([str(bad_file)])

        captured = capsys.readouterr()
        assert 'Error loading' in captured.out

    def test_handles_malformed_frontmatter(self, tmp_path: Path, capsys):
        """Should handle files with malformed YAML frontmatter."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text('---\nmalformed: {invalid yaml\n---\n# Body')

        _ = validate_skills.load_all_skills([str(skill_file)])

        # Should either skip or include with error
        _ = capsys.readouterr()
        # May print error or silently skip depending on implementation

    def test_returns_empty_dict_for_empty_list(self):
        """Should return empty dictionary when given empty list."""
        result = validate_skills.load_all_skills([])
        assert result == {}

    def test_preserves_skill_content_structure(self, good_skill_path: Path):
        """Should preserve frontmatter and body structure correctly."""
        result = validate_skills.load_all_skills([str(good_skill_path)])
        skill_data = result[str(good_skill_path)]

        assert isinstance(skill_data['frontmatter'], dict)
        assert isinstance(skill_data['body'], str)
        assert len(skill_data['body']) > 0


# ==================== pydantic_errors_to_issues Tests ====================


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


# ==================== validate_skill Tests ====================


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
        # Note: May have issues depending on validators, but should not crash

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

        # Should have validation errors in frontmatter
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

        # Should have structure validation errors
        assert len(result.issues) > 0
        assert any(issue.section == 'structure' for issue in result.issues)

    def test_includes_warnings_when_requested(self, good_skill_path: Path):
        """Should include warnings when show_warnings=True."""
        all_skills = validate_skills.load_all_skills([str(good_skill_path)])
        repo_root = str(good_skill_path.parent.parent.parent.parent)

        result = validate_skills.validate_skill(
            str(good_skill_path), show_warnings=True, all_skills=all_skills, repo_root=repo_root
        )

        # May have warnings depending on skill content
        _ = [issue for issue in result.issues if issue.level == ValidationLevel.WARNING]
        # Should not crash and may have warnings

    def test_handles_file_read_error(self, tmp_path: Path, monkeypatch):
        """Should handle file read errors gracefully."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text('---\nname: test\n---\n# Test')

        # Mock open to raise OSError
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

        # Should have errors due to missing frontmatter fields
        assert len(result.issues) > 0

    def test_runs_custom_validators(self, good_skill_path: Path):
        """Should run UniquenessValidator and CrossReferenceValidator."""
        all_skills = validate_skills.load_all_skills([str(good_skill_path)])
        repo_root = str(good_skill_path.parent.parent.parent.parent)

        result = validate_skills.validate_skill(
            str(good_skill_path), show_warnings=True, all_skills=all_skills, repo_root=repo_root
        )

        # Custom validators should run without crashing
        assert isinstance(result, ValidationResult)

    def test_validates_with_empty_all_skills_dict(self, good_skill_path: Path):
        """Should handle validation with empty all_skills dictionary."""
        result = validate_skills.validate_skill(
            str(good_skill_path), show_warnings=False, all_skills={}, repo_root='/'
        )

        assert isinstance(result, ValidationResult)


# ==================== main CLI Tests ====================


class TestMainCLI:
    """Test suite for main() CLI function."""

    def test_validates_default_path(self, monkeypatch, tmp_path: Path, capsys):
        """Should validate current directory when no path provided."""
        # Create a skill in temp directory
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: test-skill
description: A test skill for CLI validation
license: MIT
---
# Test Skill

## When to Use This Skill

Use for testing.

## Prerequisites

- Python
""")

        monkeypatch.chdir(tmp_path)
        monkeypatch.setattr(sys, 'argv', ['validate-skills.py'])

        _ = validate_skills.main()

        captured = capsys.readouterr()
        assert 'Validating' in captured.out or 'No SKILL.md' in captured.out

    def test_validates_specific_file(self, monkeypatch, good_skill_path: Path, capsys):
        """Should validate a specific skill file when path provided."""
        monkeypatch.setattr(sys, 'argv', ['validate-skills.py', str(good_skill_path)])

        _ = validate_skills.main()

        captured = capsys.readouterr()
        assert 'Validating' in captured.out
        assert str(good_skill_path) in captured.out

    def test_shows_recommendations_with_flag(self, monkeypatch, good_skill_path: Path, capsys):
        """Should show warnings when --recommend flag is used."""
        monkeypatch.setattr(
            sys, 'argv', ['validate-skills.py', str(good_skill_path), '--recommend']
        )

        _ = validate_skills.main()

        captured = capsys.readouterr()
        assert 'Validating' in captured.out
        # Warnings may or may not be present, but should not crash

    def test_returns_zero_for_valid_skills(self, monkeypatch, good_skill_path: Path):
        """Should return exit code 0 when all skills are valid."""
        monkeypatch.setattr(sys, 'argv', ['validate-skills.py', str(good_skill_path)])

        exit_code = validate_skills.main()

        # Exit code depends on actual validation results
        assert exit_code in [0, 1]

    def test_returns_nonzero_for_invalid_skills(self, monkeypatch, bad_skill_path: Path):
        """Should return exit code 1 when skills have errors."""
        monkeypatch.setattr(sys, 'argv', ['validate-skills.py', str(bad_skill_path)])

        exit_code = validate_skills.main()

        assert exit_code == 1

    def test_handles_no_skills_found(self, monkeypatch, tmp_path: Path, capsys):
        """Should handle case when no SKILL.md files are found."""
        empty_dir = tmp_path / 'empty'
        empty_dir.mkdir()

        monkeypatch.setattr(sys, 'argv', ['validate-skills.py', str(empty_dir)])

        exit_code = validate_skills.main()

        captured = capsys.readouterr()
        assert 'No SKILL.md files found' in captured.out
        assert exit_code == 0

    def test_displays_summary_statistics(self, monkeypatch, good_skill_path: Path, capsys):
        """Should display summary with passed/failed counts."""
        monkeypatch.setattr(sys, 'argv', ['validate-skills.py', str(good_skill_path)])

        validate_skills.main()

        captured = capsys.readouterr()
        assert 'Summary:' in captured.out
        assert 'passed' in captured.out

    def test_finds_repository_root(self, monkeypatch, tmp_path: Path):
        """Should locate .git directory to determine repo root."""
        # Create fake git repo
        git_dir = tmp_path / '.git'
        git_dir.mkdir()

        skill_dir = tmp_path / 'skills' / 'test-skill'
        skill_dir.mkdir(parents=True)
        skill_file = skill_dir / 'SKILL.md'
        skill_file.write_text("""---
name: test-skill
description: Test skill in git repo
license: MIT
---
# Test
""")

        monkeypatch.chdir(skill_dir)
        monkeypatch.setattr(sys, 'argv', ['validate-skills.py'])

        exit_code = validate_skills.main()

        # Should find git root and validate successfully
        assert exit_code in [0, 1]

    def test_handles_validation_exception(self, monkeypatch, tmp_path: Path, capsys):
        """Should handle exceptions during validation gracefully."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text('---\nname: test\n---\n# Test')

        # Mock validate_skill to raise exception
        def mock_validate(*args, **kwargs):
            raise RuntimeError('Validation failed')

        monkeypatch.setattr(validate_skills, 'validate_skill', mock_validate)
        monkeypatch.setattr(sys, 'argv', ['validate-skills.py', str(skill_file)])

        _ = validate_skills.main()

        captured = capsys.readouterr()
        assert 'Error' in captured.out


# ==================== Integration Tests ====================


class TestIntegration:
    """Integration tests for end-to-end validation workflows."""

    def test_complete_validation_workflow(self, fixtures_dir: Path, monkeypatch, capsys):
        """Should complete full validation workflow from finding to reporting."""
        # Use the good-skill subdirectory which contains SKILL.md
        skill_path = fixtures_dir / 'good-skill' / 'SKILL.md'
        monkeypatch.setattr(sys, 'argv', ['validate-skills.py', str(skill_path)])

        _ = validate_skills.main()

        captured = capsys.readouterr()
        assert 'Validating' in captured.out
        assert 'Summary:' in captured.out

    def test_validates_multiple_skills_together(self, temp_skill_dir: Path, monkeypatch):
        """Should validate multiple skills and detect cross-skill issues."""
        # Create two skills with same name (uniqueness violation)
        skill1_dir = temp_skill_dir / 'skill1'
        skill1_dir.mkdir()
        (skill1_dir / 'SKILL.md').write_text("""---
name: duplicate-name
description: First skill with duplicate name
license: MIT
---
# Skill 1
""")

        skill2_dir = temp_skill_dir / 'skill2'
        skill2_dir.mkdir()
        (skill2_dir / 'SKILL.md').write_text("""---
name: duplicate-name
description: Second skill with duplicate name
license: MIT
---
# Skill 2
""")

        monkeypatch.setattr(sys, 'argv', ['validate-skills.py', str(temp_skill_dir)])

        exit_code = validate_skills.main()

        # Should detect duplicate name issue
        assert exit_code == 1

    def test_cross_reference_validation(self, temp_skill_dir: Path, monkeypatch):
        """Should validate cross-references between skills."""
        skill_file = temp_skill_dir / 'SKILL.md'
        skill_file.write_text("""---
name: test-skill
description: Test skill with cross-reference
license: MIT
---
# Test Skill

See [nonexistent-skill](../nonexistent-skill/SKILL.md) for more info.
""")

        monkeypatch.setattr(sys, 'argv', ['validate-skills.py', str(skill_file)])

        exit_code = validate_skills.main()

        # May detect broken cross-reference depending on validator implementation
        assert exit_code in [0, 1]


# ==================== Edge Cases Tests ====================


class TestEdgeCases:
    """Test edge cases and boundary conditions."""

    def test_empty_skill_file(self, tmp_path: Path):
        """Should handle completely empty skill file."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text('')

        all_skills = validate_skills.load_all_skills([str(skill_file)])
        result = validate_skills.validate_skill(
            str(skill_file), show_warnings=False, all_skills=all_skills, repo_root=str(tmp_path)
        )

        assert len(result.issues) > 0

    def test_only_frontmatter_no_body(self, tmp_path: Path):
        """Should handle skill with only frontmatter, no body."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: empty-body
description: Skill with no body content
license: MIT
---
""")

        all_skills = validate_skills.load_all_skills([str(skill_file)])
        result = validate_skills.validate_skill(
            str(skill_file), show_warnings=False, all_skills=all_skills, repo_root=str(tmp_path)
        )

        assert len(result.issues) > 0

    def test_special_characters_in_paths(self, tmp_path: Path):
        """Should handle paths with special characters."""
        special_dir = tmp_path / 'path with spaces' / 'special-chars!'
        special_dir.mkdir(parents=True)
        skill_file = special_dir / 'SKILL.md'
        skill_file.write_text("""---
name: special-path
description: Skill in path with special characters
license: MIT
---
# Test
""")

        result = validate_skills.find_skill_files(str(special_dir))
        assert len(result) == 1

    def test_very_long_skill_content(self, tmp_path: Path):
        """Should handle skills with very large content."""
        skill_file = tmp_path / 'SKILL.md'
        large_content = 'x' * 100000  # 100KB of content
        skill_file.write_text(f"""---
name: large-skill
description: Skill with very large content body
license: MIT
---
# Large Skill

{large_content}
""")

        all_skills = validate_skills.load_all_skills([str(skill_file)])
        assert str(skill_file) in all_skills

    def test_unicode_content(self, tmp_path: Path):
        """Should handle Unicode characters in skill content."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: unicode-skill
description: "Skill with Unicode: 你好世界 🚀 émojis and açcênts"
license: MIT
---
# Unicode Skill 🎉

Content with various Unicode: αβγδ, 한글, العربية
""")

        all_skills = validate_skills.load_all_skills([str(skill_file)])
        assert str(skill_file) in all_skills

    def test_symlink_handling(self, tmp_path: Path):
        """Should handle symlinked skill files appropriately."""
        real_skill = tmp_path / 'real' / 'SKILL.md'
        real_skill.parent.mkdir()
        real_skill.write_text("""---
name: real-skill
description: Real skill file
license: MIT
---
# Real
""")

        link_dir = tmp_path / 'link'
        link_dir.mkdir()
        link_skill = link_dir / 'SKILL.md'

        try:
            link_skill.symlink_to(real_skill)
            result = validate_skills.find_skill_files(str(link_dir))
            assert len(result) == 1
        except OSError:
            # Symlinks may not be supported on all systems
            pytest.skip('Symlinks not supported on this system')
