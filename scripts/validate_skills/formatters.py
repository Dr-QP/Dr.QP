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

"""Output formatters for validation results."""

import csv
import io
import json
from typing import List

from validate_skills.types import ValidationLevel, ValidationResult


class TextFormatter:
    """Formats results as readable text."""

    def __init__(self, use_colors: bool = False):
        """
        Initialize the formatter.

        Args:
        ----
            use_colors:
                Whether to use ANSI color codes.

        """
        self.use_colors = use_colors

    def format(self, result: ValidationResult) -> str:
        """
        Format a single result.

        Args
        ----
            result: ValidationResult to format

        Returns
        -------
            Formatted string

        """
        lines = []
        status = '✓' if result.is_valid else '✗'
        lines.append(f'{status} {result.skill_path}')

        for issue in result.issues:
            lines.append(str(issue))

        return '\n'.join(lines)

    def format_summary(self, results: List[ValidationResult]) -> str:
        """
        Format summary statistics.

        Args
        ----
            results: List of validation results

        Returns
        -------
            Formatted summary

        """
        total = len(results)
        valid = sum(1 for r in results if r.is_valid)
        warnings = sum(
            1 for r in results for issue in r.issues if issue.level == ValidationLevel.WARNING
        )
        errors = sum(
            1 for r in results for issue in r.issues if issue.level == ValidationLevel.ERROR
        )

        lines = [
            f'\nSummary: {valid}/{total} skills valid',
            f'Errors: {errors}, Warnings: {warnings}',
        ]
        return '\n'.join(lines)

    def format_batch(self, results: List[ValidationResult]) -> str:
        """
        Format multiple results.

        Args
        ----
            results: List of validation results

        Returns
        -------
            Formatted batch output

        """
        if not results:
            return ''
        lines = [self.format(r) for r in results]
        lines.append(self.format_summary(results))
        return '\n'.join(lines)


class JSONFormatter:
    """Formats results as JSON."""

    def format(self, result: ValidationResult) -> str:
        """
        Format a single result.

        Args
        ----
            result: ValidationResult to format

        Returns
        -------
            JSON string

        """
        data = {
            'skill_path': result.skill_path,
            'is_valid': result.is_valid,
            'issues': [
                {
                    'level': issue.level.value,
                    'message': issue.message,
                    'section': issue.section,
                    'line_number': issue.line_number,
                }
                for issue in result.issues
            ],
        }
        return json.dumps(data, indent=2)

    def format_batch(self, results: List[ValidationResult]) -> str:
        """
        Format multiple results.

        Args
        ----
            results: List of validation results

        Returns
        -------
            JSON array string

        """
        data = [
            {
                'skill_path': r.skill_path,
                'is_valid': r.is_valid,
                'issues': [
                    {
                        'level': issue.level.value,
                        'message': issue.message,
                        'section': issue.section,
                        'line_number': issue.line_number,
                    }
                    for issue in r.issues
                ],
            }
            for r in results
        ]
        return json.dumps(data, indent=2)


class CSVFormatter:
    """Formats results as CSV."""

    def get_header(self) -> str:
        """
        Get CSV header row.

        Returns
        -------
            Header row string

        """
        return 'skill_path,is_valid,issue_level,issue_message,section'

    def format(self, result: ValidationResult) -> str:
        """
        Format a single result.

        Args
        ----
            result: ValidationResult to format

        Returns
        -------
            CSV row string

        """
        output = io.StringIO()
        writer = csv.writer(output)

        if not result.issues:
            writer.writerow([result.skill_path, str(result.is_valid), '', '', ''])
        else:
            for issue in result.issues:
                writer.writerow(
                    [
                        result.skill_path,
                        str(result.is_valid),
                        issue.level.value,
                        issue.message,
                        issue.section or '',
                    ]
                )

        return output.getvalue().strip()


def format_results(results: List[ValidationResult], output_format: str = 'text') -> str:
    """
    Format validation results in the requested format.

    Args
    ----
        results: List of validation results
        output_format: Format type ('text', 'json', 'csv')

    Returns
    -------
        Formatted string

    Raises
    ------
        ValueError: If output_format is not one of the supported types

    """
    if output_format not in ('text', 'json', 'csv'):
        raise ValueError(f"Invalid format '{output_format}'. Supported formats: text, json, csv")

    if output_format == 'json':
        formatter = JSONFormatter()
        return formatter.format_batch(results)
    elif output_format == 'csv':
        formatter = CSVFormatter()
        lines = [formatter.get_header()]
        for result in results:
            lines.append(formatter.format(result))
        return '\n'.join(lines)
    else:  # text
        formatter = TextFormatter()
        if not results:
            return ''
        lines = [formatter.format(r) for r in results]
        lines.append(formatter.format_summary(results))
        return '\n'.join(lines)
