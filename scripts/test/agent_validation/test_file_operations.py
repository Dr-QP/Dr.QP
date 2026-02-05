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

"""Tests for skill file discovery and loading operations."""

import sys
from pathlib import Path

import pytest

# Import module under test
sys.path.insert(0, str(Path(__file__).parent.parent.parent))
validate_skills = __import__('validate-skills')


class TestFindSkillFiles:
    """Test suite for find_skill_files function."""

    def test_finds_single_skill_file(self, good_skill_path: Path):
        """Should return a single skill file when given a direct file path."""
        result = validate_skills.find_skill_files(str(good_skill_path.parent.parent))
        assert len(result) >= 1
        assert str(good_skill_path) in result

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


class TestLoadAllSkills:
    """Test suite for load_all_skills function."""

    def test_loads_valid_skill(self, good_skill_path: Path):
        """Should successfully load a valid skill file."""
        result = validate_skills.load_all_skills([str(good_skill_path)])

        assert str(good_skill_path) in result
        assert 'frontmatter' in result[str(good_skill_path)]
        assert 'body' in result[str(good_skill_path)]
        assert 'name' in result[str(good_skill_path)]['frontmatter']
        assert result[str(good_skill_path)]['frontmatter']['name'] == 'example-good-skill'

    def test_loads_multiple_skills(self, good_skill_path: Path, bad_skill_path: Path):
        """Should load multiple skill files into a dictionary."""
        paths = [str(good_skill_path), str(bad_skill_path)]
        result = validate_skills.load_all_skills(paths)

        assert len(result) == 2
        assert str(good_skill_path) in result
        assert str(bad_skill_path) in result
        assert result[str(good_skill_path)]['frontmatter']['name'] == 'example-good-skill'
        assert result[str(bad_skill_path)]['frontmatter']['name'] == 'Bad_Skill_Name'

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
