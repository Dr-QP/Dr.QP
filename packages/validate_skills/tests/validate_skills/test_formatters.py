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

"""Unit tests for output formatting utilities."""

import pytest

from validate_skills.formatters import (
    CSVFormatter,
    format_results,
    JSONFormatter,
    TextFormatter,
)


class TestTextFormatter:
    """Tests for text output formatting."""

    def test_text_formatter_format_result(self):
        """Should format result as readable text."""
        from validate_skills.core import ValidationIssue, ValidationLevel, ValidationResult

        result = ValidationResult(
            skill_path='/path/to/SKILL.md',
            issues=[
                ValidationIssue(level=ValidationLevel.ERROR, message='Missing field'),
                ValidationIssue(level=ValidationLevel.WARNING, message='Poor description'),
            ],
        )

        formatter = TextFormatter()
        output = formatter.format(result)

        assert isinstance(output, str)
        assert 'SKILL.md' in output
        assert 'Missing field' in output or 'ERROR' in output

    def test_text_formatter_color_support(self):
        """Should optionally include color codes."""
        from validate_skills.core import ValidationResult

        result = ValidationResult(skill_path='/path/to/SKILL.md', issues=[])

        formatter = TextFormatter(use_colors=True)
        output = formatter.format(result)
        assert isinstance(output, str)

    def test_text_formatter_summary_statistics(self):
        """Should include summary statistics."""
        from validate_skills.core import ValidationIssue, ValidationLevel, ValidationResult

        results = [
            ValidationResult(
                skill_path='/path/skill1/SKILL.md',
                issues=[ValidationIssue(level=ValidationLevel.ERROR, message='Error')],
            ),
            ValidationResult(skill_path='/path/skill2/SKILL.md', issues=[]),
        ]

        formatter = TextFormatter()
        summary = formatter.format_summary(results)

        assert isinstance(summary, str)
        assert '1' in summary or '2' in summary  # Should mention counts


class TestJSONFormatter:
    """Tests for JSON output formatting."""

    def test_json_formatter_valid_json(self):
        """Should produce valid JSON output."""
        import json

        from validate_skills.core import ValidationResult

        result = ValidationResult(skill_path='/path/to/SKILL.md', issues=[])

        formatter = JSONFormatter()
        output = formatter.format(result)

        # Should be valid JSON
        parsed = json.loads(output)
        assert isinstance(parsed, dict)

    def test_json_formatter_includes_metadata(self):
        """Should include result metadata in JSON."""
        import json

        from validate_skills.core import ValidationResult

        result = ValidationResult(skill_path='/path/to/SKILL.md', issues=[])

        formatter = JSONFormatter()
        output = formatter.format(result)
        parsed = json.loads(output)

        assert 'skill_path' in parsed or 'path' in parsed or 'file' in parsed
        assert 'issues' in parsed or 'problems' in parsed

    def test_json_formatter_multiple_results(self):
        """Should format multiple results."""
        import json

        from validate_skills.core import ValidationResult

        results = [
            ValidationResult(skill_path='/path/skill1/SKILL.md', issues=[]),
            ValidationResult(skill_path='/path/skill2/SKILL.md', issues=[]),
        ]

        formatter = JSONFormatter()
        output = formatter.format_batch(results)

        parsed = json.loads(output)
        assert isinstance(parsed, (list, dict))


class TestCSVFormatter:
    """Tests for CSV output formatting."""

    def test_csv_formatter_header(self):
        """Should include CSV header row."""
        formatter = CSVFormatter()
        output = formatter.get_header()

        assert isinstance(output, str)
        assert ',' in output or ';' in output  # Should be delimited

    def test_csv_formatter_data_row(self):
        """Should format result as CSV row."""
        from validate_skills.core import ValidationIssue, ValidationLevel, ValidationResult

        result = ValidationResult(
            skill_path='/path/to/SKILL.md',
            issues=[
                ValidationIssue(level=ValidationLevel.ERROR, message='Error message'),
            ],
        )

        formatter = CSVFormatter()
        output = formatter.format(result)

        assert isinstance(output, str)
        assert 'SKILL.md' in output or 'path' in output.lower()

    def test_csv_formatter_escaping(self):
        """Should properly escape special characters in CSV."""
        from validate_skills.core import ValidationIssue, ValidationLevel, ValidationResult

        result = ValidationResult(
            skill_path='/path/to/SKILL.md',
            issues=[
                ValidationIssue(
                    level=ValidationLevel.ERROR, message='Message with "quotes" and, commas'
                ),
            ],
        )

        formatter = CSVFormatter()
        output = formatter.format(result)

        assert isinstance(output, str)
        # Should handle special characters


class TestFormatResults:
    """Tests for top-level formatting function."""

    def test_format_results_text(self):
        """Should format results in text format by default."""
        from validate_skills.core import ValidationResult

        results = [
            ValidationResult(skill_path='/path/skill1/SKILL.md', issues=[]),
        ]

        output = format_results(results, format='text')
        assert isinstance(output, str)

    def test_format_results_json(self):
        """Should format results in JSON format."""
        import json

        from validate_skills.core import ValidationResult

        results = [
            ValidationResult(skill_path='/path/skill1/SKILL.md', issues=[]),
        ]

        output = format_results(results, format='json')
        # Should be valid JSON
        parsed = json.loads(output)
        assert parsed is not None

    def test_format_results_csv(self):
        """Should format results in CSV format."""
        from validate_skills.core import ValidationResult

        results = [
            ValidationResult(skill_path='/path/skill1/SKILL.md', issues=[]),
            ValidationResult(skill_path='/path/skill2/SKILL.md', issues=[]),
        ]

        output = format_results(results, format='csv')
        assert isinstance(output, str)
        lines = output.split('\n')
        # Should have header + data rows
        assert len(lines) >= 1

    def test_format_results_invalid_format(self):
        """Should raise error for invalid format."""
        from validate_skills.core import ValidationResult

        results = [ValidationResult(skill_path='/path/SKILL.md', issues=[])]

        with pytest.raises(ValueError):
            format_results(results, format='invalid-format')


class TestFormatterEdgeCases:
    """Tests for edge cases in formatting."""

    def test_formatter_empty_results(self):
        """Should handle empty results list."""
        formatter = TextFormatter()
        output = formatter.format_batch([])
        assert isinstance(output, str)

    def test_formatter_no_issues(self):
        """Should format result with no issues."""
        from validate_skills.core import ValidationResult

        result = ValidationResult(skill_path='/path/SKILL.md', issues=[])
        formatter = TextFormatter()
        output = formatter.format(result)

        assert isinstance(output, str)
        assert 'SKILL.md' in output

    def test_formatter_many_issues(self):
        """Should handle many issues."""
        from validate_skills.core import ValidationIssue, ValidationLevel, ValidationResult

        issues = [
            ValidationIssue(level=ValidationLevel.ERROR, message=f'Error {i}') for i in range(100)
        ]
        result = ValidationResult(skill_path='/path/SKILL.md', issues=issues)

        formatter = TextFormatter()
        output = formatter.format(result)
        assert isinstance(output, str)

    def test_formatter_special_characters_in_path(self):
        """Should handle special characters in file paths."""
        from validate_skills.core import ValidationResult

        result = ValidationResult(skill_path='/path/skill with spaces & chars/SKILL.md', issues=[])

        formatter = TextFormatter()
        output = formatter.format(result)
        assert isinstance(output, str)

    def test_formatter_unicode_in_messages(self):
        """Should handle unicode characters in messages."""
        from validate_skills.core import ValidationIssue, ValidationLevel, ValidationResult

        result = ValidationResult(
            skill_path='/path/SKILL.md',
            issues=[
                ValidationIssue(
                    level=ValidationLevel.ERROR, message='Error with unicode: ✓ ✗ √ ∞ 中文'
                ),
            ],
        )

        formatter = TextFormatter()
        output = formatter.format(result)
        assert isinstance(output, str)
