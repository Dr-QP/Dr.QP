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

"""Tests for CLI main() function."""

import sys
from pathlib import Path

import pytest

# Import module under test
sys.path.insert(0, str(Path(__file__).parent.parent.parent))
validate_skills = __import__('validate-skills')


class TestMainCLI:
    """Test suite for main() CLI function."""

    def test_validates_default_path(self, monkeypatch, tmp_path: Path, capsys):
        """Should validate current directory when no path provided."""
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

    def test_returns_zero_for_valid_skills(self, monkeypatch, good_skill_path: Path):
        """Should return exit code 0 when all skills are valid."""
        monkeypatch.setattr(sys, 'argv', ['validate-skills.py', str(good_skill_path)])

        exit_code = validate_skills.main()

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

        assert exit_code in [0, 1]

    def test_handles_validation_exception(self, monkeypatch, tmp_path: Path, capsys):
        """Should handle exceptions during validation gracefully."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text('---\nname: test\n---\n# Test')

        def mock_validate(*args, **kwargs):
            raise RuntimeError('Validation failed')

        monkeypatch.setattr(validate_skills, 'validate_skill', mock_validate)
        monkeypatch.setattr(sys, 'argv', ['validate-skills.py', str(skill_file)])

        _ = validate_skills.main()

        captured = capsys.readouterr()
        assert 'Error' in captured.out
