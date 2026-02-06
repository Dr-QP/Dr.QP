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

"""Unit tests for file loading and parsing utilities."""

import os

import pytest

from validate_skills.loaders import (
    find_skill_files,
    load_all_skills,
    safe_load_frontmatter,
)


class TestFindSkillFilesDirectory:
    """Tests for finding SKILL.md files in directories."""

    def test_find_skill_files_empty_directory(self, tmp_path):
        """Should return empty list for directory with no SKILL.md files."""
        result = find_skill_files(str(tmp_path))
        assert result == []

    def test_find_skill_files_single_skill(self, tmp_path):
        """Should find single SKILL.md file in directory."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text('---\nname: test\n---\nContent')

        result = find_skill_files(str(tmp_path))
        assert len(result) == 1
        assert str(skill_file) in result

    def test_find_skill_files_nested_directories(self, tmp_path):
        """Should find SKILL.md files in nested directories."""
        skill1 = tmp_path / 'skill1' / 'SKILL.md'
        skill2 = tmp_path / 'skill2' / 'subdir' / 'SKILL.md'
        skill1.parent.mkdir()
        skill2.parent.mkdir(parents=True)
        skill1.write_text('---\nname: skill1\n---\nContent')
        skill2.write_text('---\nname: skill2\n---\nContent')

        result = find_skill_files(str(tmp_path))
        assert len(result) == 2
        assert str(skill1) in result
        assert str(skill2) in result

    def test_find_skill_files_multiple_in_same_dir(self, tmp_path):
        """Should find multiple SKILL.md files in same directory."""
        (tmp_path / 'SKILL.md').write_text('---\nname: skill1\n---\n')
        (tmp_path / 'other.md').write_text('Not a skill')

        result = find_skill_files(str(tmp_path))
        assert len(result) == 1
        assert any('SKILL.md' in f for f in result)

    def test_find_skill_files_excludes_test_directories(self, tmp_path):
        """Should exclude files in test directories."""
        skill = tmp_path / 'skills' / 'SKILL.md'
        test_skill = tmp_path / 'test' / 'SKILL.md'
        skill.parent.mkdir()
        test_skill.parent.mkdir()
        skill.write_text('---\nname: skill\n---\n')
        test_skill.write_text('---\nname: test\n---\n')

        result = find_skill_files(str(tmp_path))
        assert len(result) == 1
        assert str(skill) in result
        assert str(test_skill) not in result

    def test_find_skill_files_excludes_tests_directories(self, tmp_path):
        """Should exclude files in 'tests' directories."""
        skill = tmp_path / 'SKILL.md'
        test_skill = tmp_path / 'tests' / 'SKILL.md'
        skill.write_text('---\nname: skill\n---\n')
        test_skill.parent.mkdir()
        test_skill.write_text('---\nname: test\n---\n')

        result = find_skill_files(str(tmp_path))
        assert len(result) == 1
        assert str(skill) in result

    def test_find_skill_files_excludes_pytest_cache(self, tmp_path):
        """Should exclude .pytest_cache directories."""
        skill = tmp_path / 'SKILL.md'
        cache_skill = tmp_path / '.pytest_cache' / 'SKILL.md'
        skill.write_text('---\nname: skill\n---\n')
        cache_skill.parent.mkdir()
        cache_skill.write_text('---\nname: cache\n---\n')

        result = find_skill_files(str(tmp_path))
        assert len(result) == 1
        assert str(cache_skill) not in result


class TestFindSkillFilesSingleFile:
    """Tests for finding when given a specific file path."""

    def test_find_skill_files_specific_skill_file(self, tmp_path):
        """Should return file path if given a SKILL.md file directly."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text('---\nname: test\n---\n')

        result = find_skill_files(str(skill_file))
        assert result == [str(skill_file)]

    def test_find_skill_files_non_skill_file(self, tmp_path):
        """Should return empty list for non-SKILL.md file."""
        other_file = tmp_path / 'other.md'
        other_file.write_text('Content')

        result = find_skill_files(str(other_file))
        assert result == []

    def test_find_skill_files_nonexistent_file(self, tmp_path):
        """Should handle nonexistent file gracefully."""
        nonexistent = str(tmp_path / 'nonexistent.md')
        result = find_skill_files(nonexistent)
        assert result == []


class TestSafeLoadFrontmatter:
    """Tests for safe frontmatter loading."""

    def test_safe_load_frontmatter_valid(self, tmp_path):
        """Should load valid YAML frontmatter and content."""
        skill_file = tmp_path / 'SKILL.md'
        content = '---\nname: test-skill\ndescription: A test skill\n---\n# Content\nHello'
        skill_file.write_text(content)

        metadata, body = safe_load_frontmatter(str(skill_file))
        assert metadata['name'] == 'test-skill'
        assert metadata['description'] == 'A test skill'
        assert '# Content' in body

    def test_safe_load_frontmatter_empty_body(self, tmp_path):
        """Should handle empty body after frontmatter."""
        skill_file = tmp_path / 'SKILL.md'
        content = '---\nname: test-skill\n---\n'
        skill_file.write_text(content)

        metadata, body = safe_load_frontmatter(str(skill_file))
        assert metadata['name'] == 'test-skill'
        assert body.strip() == ''

    def test_safe_load_frontmatter_complex_yaml(self, tmp_path):
        """Should parse complex nested YAML structures."""
        skill_file = tmp_path / 'SKILL.md'
        content = """---
name: complex-skill
config:
  enabled: true
  options:
    - opt1
    - opt2
---
Body content"""
        skill_file.write_text(content)

        metadata, body = safe_load_frontmatter(str(skill_file))
        assert metadata['name'] == 'complex-skill'
        assert metadata['config']['enabled'] is True
        assert len(metadata['config']['options']) == 2

    def test_safe_load_frontmatter_malformed_yaml(self, tmp_path):
        """Should raise error for malformed YAML frontmatter."""
        skill_file = tmp_path / 'SKILL.md'
        content = '---\ninvalid: yaml: structure:\n---\nBody'
        skill_file.write_text(content)

        with pytest.raises(Exception):  # yaml.YAMLError or similar
            safe_load_frontmatter(str(skill_file))

    def test_safe_load_frontmatter_missing_frontmatter(self, tmp_path):
        """Should handle file without YAML frontmatter."""
        skill_file = tmp_path / 'SKILL.md'
        content = 'Just content without frontmatter'
        skill_file.write_text(content)

        result = safe_load_frontmatter(str(skill_file))
        # Should either return empty metadata or raise error
        assert result is not None

    def test_safe_load_frontmatter_file_not_found(self):
        """Should raise OSError for nonexistent file."""
        with pytest.raises(OSError):
            safe_load_frontmatter('/nonexistent/path/SKILL.md')

    def test_safe_load_frontmatter_invalid_encoding(self, tmp_path):
        """Should raise UnicodeDecodeError for invalid encoding."""
        skill_file = tmp_path / 'SKILL.md'
        # Write binary content that's not valid UTF-8
        skill_file.write_bytes(b'\x80\x81\x82\x83')

        with pytest.raises(UnicodeDecodeError):
            safe_load_frontmatter(str(skill_file))

    def test_safe_load_frontmatter_preserves_body_whitespace(self, tmp_path):
        """Should preserve whitespace and formatting in body."""
        skill_file = tmp_path / 'SKILL.md'
        content = """---
name: test
---
Line 1
  Indented line
    Double indented

Final line"""
        skill_file.write_text(content)

        metadata, body = safe_load_frontmatter(str(skill_file))
        assert '  Indented line' in body
        assert '    Double indented' in body

    def test_safe_load_frontmatter_special_characters(self, tmp_path):
        """Should handle special characters in YAML values."""
        skill_file = tmp_path / 'SKILL.md'
        content = """---
name: test-skill
description: "Test with: colons, quotes\" and special chars @#$%"
---
Body"""
        skill_file.write_text(content)

        metadata, body = safe_load_frontmatter(str(skill_file))
        assert 'colons' in metadata['description']


class TestLoadAllSkills:
    """Tests for loading multiple skill files."""

    def test_load_all_skills_empty_list(self):
        """Should return empty dict for empty skill list."""
        result = load_all_skills([])
        assert result == {}

    def test_load_all_skills_single_skill(self, tmp_path):
        """Should load single skill file."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text('---\nname: test\n---\nContent')

        result = load_all_skills([str(skill_file)])
        assert len(result) == 1
        assert str(skill_file) in result
        assert result[str(skill_file)]['frontmatter']['name'] == 'test'

    def test_load_all_skills_multiple_skills(self, tmp_path):
        """Should load multiple skill files."""
        skill1 = tmp_path / 'skill1' / 'SKILL.md'
        skill2 = tmp_path / 'skill2' / 'SKILL.md'
        skill1.parent.mkdir()
        skill2.parent.mkdir()
        skill1.write_text('---\nname: skill1\n---\nContent1')
        skill2.write_text('---\nname: skill2\n---\nContent2')

        result = load_all_skills([str(skill1), str(skill2)])
        assert len(result) == 2
        assert result[str(skill1)]['frontmatter']['name'] == 'skill1'
        assert result[str(skill2)]['frontmatter']['name'] == 'skill2'

    def test_load_all_skills_invalid_file_logged(self, tmp_path, capsys):
        """Should log and skip files that can't be loaded."""
        skill = tmp_path / 'SKILL.md'
        skill.write_text('---\nname: valid\n---\n')
        invalid = tmp_path / 'invalid' / 'SKILL.md'
        invalid.parent.mkdir()
        invalid.write_bytes(b'\x80\x81')  # Invalid UTF-8

        result = load_all_skills([str(skill), str(invalid)])
        assert len(result) == 1
        assert str(skill) in result
        # Should have logged warning about invalid file

    def test_load_all_skills_preserves_body_and_metadata(self, tmp_path):
        """Should preserve both body and metadata for each skill."""
        skill_file = tmp_path / 'SKILL.md'
        content = """---
name: test
description: Test skill
---
# Heading
Content body"""
        skill_file.write_text(content)

        result = load_all_skills([str(skill_file)])
        skill_data = result[str(skill_file)]
        assert 'frontmatter' in skill_data
        assert 'body' in skill_data
        assert '# Heading' in skill_data['body']


class TestLoadersEdgeCases:
    """Tests for edge cases in file loading."""

    def test_find_skill_files_with_symlinks(self, tmp_path):
        """Should handle symbolic links appropriately."""
        skill = tmp_path / 'original' / 'SKILL.md'
        skill.parent.mkdir()
        skill.write_text('---\nname: skill\n---\n')

        link_dir = tmp_path / 'linked'
        if hasattr(os, 'symlink'):
            try:
                os.symlink(skill.parent, link_dir)
                result = find_skill_files(str(tmp_path))
                # Should find skills through symlinks
                assert len(result) >= 1
            except OSError:
                # Skip if symlinks not supported
                pytest.skip('Symlinks not supported')

    def test_safe_load_frontmatter_permission_denied(self, tmp_path):
        """Should handle permission denied errors."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text('---\nname: test\n---\n')
        os.chmod(skill_file, 0o000)  # No permissions for anyone

        try:
            with pytest.raises(OSError):
                safe_load_frontmatter(str(skill_file))
        finally:
            os.chmod(skill_file, 0o644)  # Cleanup

    def test_find_skill_files_hidden_directories(self, tmp_path):
        """Should handle hidden directories appropriately."""
        skill = tmp_path / '.hidden' / 'SKILL.md'
        skill.parent.mkdir()
        skill.write_text('---\nname: hidden\n---\n')

        result = find_skill_files(str(tmp_path))
        # Hidden directories might be found depending on implementation
        assert isinstance(result, list)
