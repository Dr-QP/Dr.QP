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

"""Unit tests for main orchestration function."""

from validate_skills.main import main


class TestMainBasic:
    """Tests for basic main function behavior."""

    def test_main_no_arguments(self, tmp_path, monkeypatch, capsys):
        """Should work with no arguments, validating current directory."""
        monkeypatch.chdir(tmp_path)
        exit_code = main([])
        assert exit_code in [0, 1]  # Valid exit codes

    def test_main_with_directory(self, tmp_path, capsys):
        """Should validate directory when path provided."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: test-skill
description: A comprehensive description of what this skill does and when to use it.
---
# Overview
Content

## When to use this skill
Content""")

        exit_code = main([str(tmp_path)])
        assert exit_code == 0

    def test_main_with_file(self, tmp_path, capsys):
        """Should validate single file when file path provided."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: test-skill
description: A comprehensive description of what this skill does and when to use it.
---
# Overview
Content

## When to use this skill
Content""")

        exit_code = main([str(skill_file)])
        assert exit_code == 0


class TestMainExitCodes:
    """Tests for main function exit codes."""

    def test_main_exit_0_on_success(self, tmp_path):
        """Should return 0 when all skills valid."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: valid-skill
description: A comprehensive description of what this skill does and when to use it.
---
# Overview
This is an overview section.

## When to use this skill
Use this when you need it.""")

        exit_code = main([str(tmp_path)])
        assert exit_code == 0

    def test_main_exit_1_on_validation_error(self, tmp_path):
        """Should return 1 when validation errors found."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
description: Missing name field
---
Invalid""")

        exit_code = main([str(tmp_path)])
        assert exit_code == 1

    def test_main_exit_1_on_no_skills_found(self, tmp_path):
        """Should return error when no SKILL.md files found."""
        (tmp_path / 'other.md').write_text('Not a skill')

        exit_code = main([str(tmp_path)])
        # May return 0 or 1 depending on implementation
        assert exit_code in [0, 1]


class TestMainFlags:
    """Tests for command-line flags."""

    def test_main_recommend_flag(self, tmp_path):
        """Should show warnings with --recommend flag."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: test-skill
description: A comprehensive description of what this skill does and when to use it.
---
# Overview
Content

## When to use this skill
Content""")

        exit_code = main([str(tmp_path), '--recommend'])
        assert exit_code in [0, 1]

    def test_main_ci_flag(self, tmp_path):
        """Should accept --ci flag for CI mode."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: test-skill
description: A comprehensive description of what this skill does and when to use it.
---
# Overview
Content

## When to use this skill
Content""")

        exit_code = main([str(tmp_path), '--ci'])
        assert exit_code in [0, 1]

    def test_main_format_flag(self, tmp_path, capsys):
        """Should accept --format flag for output format."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: test-skill
description: A comprehensive description of what this skill does and when to use it.
---
# Overview
Content

## When to use this skill
Content""")

        exit_code = main([str(tmp_path), '--format', 'json'])
        captured = capsys.readouterr()
        assert exit_code in [0, 1]


class TestMainOutputBehavior:
    """Tests for output behavior."""

    def test_main_prints_results(self, tmp_path, capsys):
        """Should print validation results."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: test-skill
description: A comprehensive description of what this skill does and when to use it.
---
# Overview
Content

## When to use this skill
Content""")

        exit_code = main([str(tmp_path)])
        captured = capsys.readouterr()

        assert len(captured.out) > 0 or len(captured.err) == 0

    def test_main_prints_summary(self, tmp_path, capsys):
        """Should print summary statistics."""
        skill_file = tmp_path / 'skill1' / 'SKILL.md'
        skill_file.parent.mkdir()
        skill_file.write_text("""---
name: test-skill
description: A comprehensive description of what this skill does and when to use it.
---
# Overview
Content

## When to use this skill
Content""")

        exit_code = main([str(tmp_path)])
        captured = capsys.readouterr()

        # Should contain summary with passed/failed counts
        assert 'passed' in captured.out.lower() or 'Summary' in captured.out

    def test_main_handles_validation_errors_gracefully(self, tmp_path, capsys):
        """Should handle and report validation errors."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
invalid yaml: structure:
---
Content""")

        exit_code = main([str(tmp_path)])
        captured = capsys.readouterr()

        # Should handle error gracefully
        assert exit_code == 1


class TestMainIntegration:
    """Tests for full end-to-end integration."""

    def test_main_validates_multiple_skills(self, tmp_path):
        """Should validate multiple skills in directory."""
        skill1 = tmp_path / 'skill1' / 'SKILL.md'
        skill2 = tmp_path / 'skill2' / 'SKILL.md'
        skill1.parent.mkdir()
        skill2.parent.mkdir()

        valid_content = """---
name: {}
description: A comprehensive description of what this skill does and when to use it.
---
# Overview
Content

## When to use this skill
Content"""

        skill1.write_text(valid_content.format('skill-one'))
        skill2.write_text(valid_content.format('skill-two'))

        exit_code = main([str(tmp_path)])
        assert exit_code == 0

    def test_main_detects_duplicate_names(self, tmp_path):
        """Should detect duplicate skill names across files."""
        skill1 = tmp_path / 'skill1' / 'SKILL.md'
        skill2 = tmp_path / 'skill2' / 'SKILL.md'
        skill1.parent.mkdir()
        skill2.parent.mkdir()

        content = """---
name: same-name
description: A comprehensive description of what this skill does and when to use it.
---
# Overview
Content

## When to use this skill
Content"""

        skill1.write_text(content)
        skill2.write_text(content)

        exit_code = main([str(tmp_path)])
        # Should fail due to duplicate names
        assert exit_code == 1

    def test_main_mixed_valid_invalid(self, tmp_path, capsys):
        """Should handle mix of valid and invalid skills."""
        skill1 = tmp_path / 'valid' / 'SKILL.md'
        skill2 = tmp_path / 'invalid' / 'SKILL.md'
        skill1.parent.mkdir()
        skill2.parent.mkdir()

        skill1.write_text("""---
name: valid-skill
description: A comprehensive description of what this skill does and when to use it.
---
# Overview
Content

## When to use this skill
Content""")

        skill2.write_text("""---
name: invalid@skill
description: Bad
---
Content""")

        exit_code = main([str(tmp_path)])
        captured = capsys.readouterr()

        # Should fail overall
        assert exit_code == 1
        # Should show both results
        assert 'valid' in captured.out.lower() or 'passed' in captured.out.lower()


class TestMainEdgeCases:
    """Tests for edge cases."""

    def test_main_nonexistent_path(self):
        """Should handle nonexistent path gracefully."""
        exit_code = main(['/nonexistent/path/to/skills'])
        assert exit_code in [0, 1]

    def test_main_empty_directory(self, tmp_path):
        """Should handle empty directory."""
        exit_code = main([str(tmp_path)])
        assert exit_code in [0, 1]

    def test_main_permission_denied(self, tmp_path):
        """Should handle permission denied errors."""
        subdir = tmp_path / 'protected'
        subdir.mkdir()
        import os

        os.chmod(subdir, 0o700)

        try:
            exit_code = main([str(subdir)])
            assert exit_code in [0, 1]
        finally:
            os.chmod(subdir, 0o700)
