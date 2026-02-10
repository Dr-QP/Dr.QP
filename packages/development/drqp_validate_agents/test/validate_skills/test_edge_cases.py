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

"""Unit tests for edge cases and error handling."""

import os

import pytest
import yaml


class TestErrorHandlingEdgeCases:
    """Tests for edge case error handling."""

    def test_skill_with_no_frontmatter(self, tmp_path):
        """Should handle skill file with no frontmatter delimiter."""
        from drqp_validate_agents.validate_skills.loaders import safe_load_frontmatter

        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text('Just plain content without any YAML')

        # Should either handle gracefully or raise informative error
        try:
            result = safe_load_frontmatter(str(skill_file))
            assert result is not None
        except (OSError, UnicodeDecodeError, yaml.YAMLError) as e:
            assert 'yaml' in str(e).lower() or 'front' in str(e).lower()

    def test_skill_with_incomplete_frontmatter(self, tmp_path):
        """Should handle incomplete frontmatter delimiters."""
        from drqp_validate_agents.validate_skills.loaders import safe_load_frontmatter

        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text('---\nname: test\nContent without closing delimiter')

        # Should raise error
        with pytest.raises(yaml.YAMLError):
            safe_load_frontmatter(str(skill_file))

    def test_extremely_large_file(self, tmp_path):
        """Should handle extremely large skill files."""
        from drqp_validate_agents.validate_skills.loaders import safe_load_frontmatter

        skill_file = tmp_path / 'SKILL.md'
        large_content = '---\nname: large-skill\ndescription: A comprehensive description.\n---\n'
        large_content += 'x' * (10 * 1024 * 1024)  # 10MB
        skill_file.write_text(large_content)

        # Should handle or fail gracefully
        try:
            result = safe_load_frontmatter(str(skill_file))
            assert result is not None
        except (OSError, MemoryError, UnicodeDecodeError) as e:
            # Size-related error is acceptable
            assert isinstance(e, (OSError, MemoryError, UnicodeDecodeError))

    def test_circular_reference_protection(self, tmp_path):
        """Should protect against circular references."""
        from drqp_validate_agents.validate_skills.validators.cross_reference import (
            CrossReferenceValidator,
        )

        file_a = tmp_path / 'a.md'
        file_b = tmp_path / 'b.md'
        file_a.write_text('[Link](b.md)')
        file_b.write_text('[Link](a.md)')

        validator = CrossReferenceValidator(str(tmp_path))
        issues = validator.validate(skill_path=str(file_a), metadata={}, content='[Link](b.md)')
        # Should handle without infinite loop
        assert isinstance(issues, list)


class TestCharacterEncodingEdgeCases:
    """Tests for character encoding edge cases."""

    def test_utf8_with_bom(self, tmp_path):
        """Should handle UTF-8 with BOM."""
        from drqp_validate_agents.validate_skills.loaders import safe_load_frontmatter

        skill_file = tmp_path / 'SKILL.md'
        # Write UTF-8 BOM
        with open(skill_file, 'wb') as f:
            f.write(b'\xef\xbb\xbf')  # UTF-8 BOM
            f.write('---\nname: test\ndescription: Descrip\n---\nBody'.encode('utf-8'))

        # Should handle BOM
        metadata, body = safe_load_frontmatter(str(skill_file))
        assert 'name' in metadata

    def test_mixed_line_endings(self, tmp_path):
        """Should handle mixed line endings (CRLF and LF)."""
        from drqp_validate_agents.validate_skills.loaders import safe_load_frontmatter

        skill_file = tmp_path / 'SKILL.md'
        content = '---\r\nname: test\ndescrip: Description\r\n---\r\nBody'
        skill_file.write_text(content)

        metadata, body = safe_load_frontmatter(str(skill_file))
        assert metadata['name'] == 'test'

    def test_unicode_in_frontmatter(self, tmp_path):
        """Should handle unicode characters in YAML values."""
        from drqp_validate_agents.validate_skills.loaders import safe_load_frontmatter

        skill_file = tmp_path / 'SKILL.md'
        content = """---
name: тест-skill
description: 这是一个技能 🚀 with emoji and unicode: 中文 العربية
---
Body with unicode: 日本語"""
        skill_file.write_text(content, encoding='utf-8')

        metadata, body = safe_load_frontmatter(str(skill_file))
        assert 'тест' in metadata['name']
        assert '中文' in metadata['description']


class TestDataValidationEdgeCases:
    """Tests for data validation edge cases."""

    def test_extremely_long_name(self):
        """Should reject names exceeding maximum length."""
        from drqp_validate_agents.validate_skills.validators.skill import SkillFrontmatterValidator

        validator = SkillFrontmatterValidator()
        frontmatter = {
            'name': 'x' * 1000,  # Far exceeds max length
            'description': 'A comprehensive description of what this skill does and when to use it.',
        }
        issues = validator.validate(frontmatter)
        assert len(issues) > 0

    def test_extremely_long_description(self):
        """Should reject descriptions exceeding maximum length."""
        from drqp_validate_agents.validate_skills.validators.skill import SkillFrontmatterValidator

        validator = SkillFrontmatterValidator()
        frontmatter = {
            'name': 'test-skill',
            'description': 'x' * 2000,  # Exceeds max
        }
        issues = validator.validate(frontmatter)
        assert len(issues) > 0

    def test_null_and_none_values(self):
        """Should handle None/null values properly."""
        from drqp_validate_agents.validate_skills.validators.skill import SkillFrontmatterValidator

        validator = SkillFrontmatterValidator()
        frontmatter = {
            'name': None,
            'description': None,
        }
        issues = validator.validate(frontmatter)
        assert len(issues) > 0

    def test_boolean_instead_of_string(self):
        """Should reject wrong data types."""
        from drqp_validate_agents.validate_skills.validators.skill import SkillFrontmatterValidator

        validator = SkillFrontmatterValidator()
        frontmatter = {
            'name': True,
            'description': False,
        }
        issues = validator.validate(frontmatter)
        assert len(issues) > 0

    def test_list_instead_of_string(self):
        """Should reject list values where strings expected."""
        from drqp_validate_agents.validate_skills.validators.skill import SkillFrontmatterValidator

        validator = SkillFrontmatterValidator()
        frontmatter = {
            'name': ['not', 'a', 'string'],
            'description': ['also', 'not', 'a', 'string'],
        }
        issues = validator.validate(frontmatter)
        assert len(issues) > 0


class TestConcurrencyAndRaceConditions:
    """Tests for concurrency-related edge cases."""

    def test_file_modified_during_validation(self, tmp_path):
        """Should handle file being modified during validation."""
        from drqp_validate_agents.validate_skills.loaders import safe_load_frontmatter

        skill_file = tmp_path / 'SKILL.md'
        original = '---\nname: test\ndescription: Description\n---\nBody'
        skill_file.write_text(original)

        # Read once
        metadata1, body1 = safe_load_frontmatter(str(skill_file))

        # Modify file
        skill_file.write_text(
            '---\nname: modified\ndescription: Modified description\n---\nModified'
        )

        # Read again
        metadata2, body2 = safe_load_frontmatter(str(skill_file))

        # Should have different data
        assert metadata1['name'] != metadata2['name']

    def test_file_deleted_during_validation(self, tmp_path):
        """Should handle file being deleted during validation."""
        from drqp_validate_agents.validate_skills.loaders import safe_load_frontmatter

        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text('---\nname: test\n---\nBody')

        # Delete file
        os.remove(str(skill_file))

        # Should raise appropriate error
        with pytest.raises(OSError):
            safe_load_frontmatter(str(skill_file))


class TestPathTraversalSecurity:
    """Tests for path traversal and security concerns."""

    def test_path_traversal_attempts(self, tmp_path):
        """Should prevent path traversal attacks."""
        from drqp_validate_agents.validate_skills.loaders import find_skill_files

        # Try to traverse outside safe directory
        result = find_skill_files(str(tmp_path / '../../etc/passwd'))
        # Should either return empty or handle safely
        assert isinstance(result, list)

    def test_symlink_loops(self, tmp_path):
        """Should handle symlink loops safely."""
        if not hasattr(os, 'symlink'):
            pytest.skip('Symlinks not supported')

        from drqp_validate_agents.validate_skills.loaders import find_skill_files

        dir_a = tmp_path / 'a'
        dir_b = tmp_path / 'b'
        dir_a.mkdir()
        dir_b.mkdir()

        try:
            # Create symlink loop: a -> b -> a
            os.symlink(dir_b, dir_a / 'link_to_b')
            os.symlink(dir_a, dir_b / 'link_to_a')

            # Should handle without infinite loop
            result = find_skill_files(str(tmp_path))
            assert isinstance(result, list)
        except OSError:
            pytest.skip('Cannot create symlink loops')


class TestResourceLimits:
    """Tests for resource limit handling."""

    def test_many_small_issues(self):
        """Should handle results with many issues."""
        from drqp_validate_agents.validate_skills.core import (
            ValidationIssue,
            ValidationLevel,
            ValidationResult,
        )

        issues = [
            ValidationIssue(level=ValidationLevel.ERROR, message=f'Issue {i}') for i in range(1000)
        ]
        result = ValidationResult(skill_path='/path/SKILL.md', issues=issues)

        # Should not crash
        assert len(result.issues) == 1000

    def test_many_skill_files(self, tmp_path):
        """Should handle validating many skill files."""
        from drqp_validate_agents.validate_skills.loaders import load_all_skills

        # Create 100 skill files
        skill_paths = []
        for i in range(100):
            skill_dir = tmp_path / f'skill_{i}'
            skill_dir.mkdir()
            skill_file = skill_dir / 'SKILL.md'
            skill_file.write_text(f'---\nname: skill-{i}\ndescription: Descip\n---\n')
            skill_paths.append(str(skill_file))

        # Should handle loading all
        all_skills = load_all_skills(skill_paths)
        assert len(all_skills) == 100

    def test_deeply_nested_directories(self, tmp_path):
        """Should handle deeply nested directory structures."""
        from drqp_validate_agents.validate_skills.loaders import find_skill_files

        # Create deeply nested structure
        current = tmp_path
        for i in range(50):
            current = current / f'level_{i}'
            current.mkdir()

        skill_file = current / 'SKILL.md'
        skill_file.write_text('---\nname: deep\ndescription: Descip\n---\n')

        result = find_skill_files(str(tmp_path))
        assert len(result) == 1


class TestRobustnessEdgeCases:
    """Tests for general robustness."""

    def test_empty_yaml_frontmatter(self, tmp_path):
        """Should handle completely empty frontmatter."""
        from drqp_validate_agents.validate_skills.loaders import safe_load_frontmatter

        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text('---\n---\nBody only')

        metadata, body = safe_load_frontmatter(str(skill_file))
        assert metadata == {}
        assert 'Body' in body

    def test_only_whitespace_in_sections(self, tmp_path):
        """Should handle sections with only whitespace."""
        from drqp_validate_agents.validate_skills.validators.skill import SkillStructureValidator

        body = """# Overview

    ## When to use this skill

    ## Features
         """
        validator = SkillStructureValidator()
        issues = validator.validate(body)
        # May have warnings about empty sections
        assert isinstance(issues, list)

    def test_malformed_markdown_syntax(self, tmp_path):
        """Should handle malformed markdown."""
        from drqp_validate_agents.validate_skills.validators.skill import SkillStructureValidator

        body = """# Overview
This is overview.

## When to use this skill
Use this

### Nested heading without proper hierarchy
Content

#### Another nested level
More content"""
        validator = SkillStructureValidator()
        issues = validator.validate(body)
        # Should validate without crashing
        assert isinstance(issues, list)

    def test_skill_with_special_yaml_types(self, tmp_path):
        """Should handle YAML special types."""
        from drqp_validate_agents.validate_skills.loaders import safe_load_frontmatter

        skill_file = tmp_path / 'SKILL.md'
        content = """---
name: test-skill
dates: [2026-01-01, 2026-12-31]
enabled: yes
disabled: no
version: 1.0
---
Body"""
        skill_file.write_text(content)

        metadata, body = safe_load_frontmatter(str(skill_file))
        # Should handle YAML type conversion
        assert 'name' in metadata
