#!/usr/bin/env python3

"""Output formatters for validation results."""

import csv
import io
import json
from typing import List

from .types import ValidationIssue, ValidationLevel, ValidationResult


class TextFormatter:
    """Formats results as readable text."""

    def __init__(self, use_colors: bool = False):
        self.use_colors = use_colors

    def format_result(self, result: ValidationResult) -> str:
        lines = []
        status = '✓' if result.is_valid else '✗'
        lines.append(f'{status} {result.skill_path}')

        for issue in result.issues:
            lines.append(self._format_issue(result.skill_path, issue))

        return '\n'.join(lines)

    def _format_issue(self, skill_path: str, issue: ValidationIssue) -> str:
        section_info = f' [{issue.section}]' if issue.section else ''

        if issue.line_number and issue.column_number:
            return (
                f'{skill_path}:{issue.line_number}:{issue.column_number}: '
                f'{issue.message}{section_info}'
            )

        if issue.line_number:
            return f'{skill_path}:{issue.line_number}: {issue.message}{section_info}'

        return f'{skill_path}: {issue.message}{section_info}'

    def format_summary(self, results: List[ValidationResult]) -> str:
        total = len(results)
        valid = sum(1 for result in results if result.is_valid)
        warnings = sum(
            1
            for result in results
            for issue in result.issues
            if issue.level == ValidationLevel.WARNING
        )
        errors = sum(
            1
            for result in results
            for issue in result.issues
            if issue.level == ValidationLevel.ERROR
        )

        lines = [
            f'\nSummary: {valid}/{total} skills valid',
            f'Errors: {errors}, Warnings: {warnings}',
        ]
        return '\n'.join(lines)

    def format_batch(self, results: List[ValidationResult]) -> str:
        if not results:
            return ''
        lines = [self.format_result(result) for result in results]
        lines.append(self.format_summary(results))
        return '\n'.join(lines)


class JSONFormatter:
    """Formats results as JSON."""

    def format_result(self, result: ValidationResult) -> str:
        data = {
            'skill_path': result.skill_path,
            'is_valid': result.is_valid,
            'issues': [
                {
                    'level': issue.level.value,
                    'message': issue.message,
                    'section': issue.section,
                    'line_number': issue.line_number,
                    'column_number': issue.column_number,
                }
                for issue in result.issues
            ],
        }
        return json.dumps(data, indent=2)

    def format_batch(self, results: List[ValidationResult]) -> str:
        data = [
            {
                'skill_path': result.skill_path,
                'is_valid': result.is_valid,
                'issues': [
                    {
                        'level': issue.level.value,
                        'message': issue.message,
                        'section': issue.section,
                        'line_number': issue.line_number,
                        'column_number': issue.column_number,
                    }
                    for issue in result.issues
                ],
            }
            for result in results
        ]
        return json.dumps(data, indent=2)


class CSVFormatter:
    """Formats results as CSV."""

    def get_header(self) -> str:
        return 'skill_path,is_valid,issue_level,issue_message,section,line_number,column_number'

    def format_result(self, result: ValidationResult) -> str:
        output = io.StringIO()
        writer = csv.writer(output)

        if not result.issues:
            writer.writerow([result.skill_path, str(result.is_valid), '', '', '', '', ''])
        else:
            for issue in result.issues:
                writer.writerow(
                    [
                        result.skill_path,
                        str(result.is_valid),
                        issue.level.value,
                        issue.message,
                        issue.section or '',
                        issue.line_number or '',
                        issue.column_number or '',
                    ]
                )

        return output.getvalue().strip()


def format_results(
    results: List[ValidationResult],
    output_format: str = 'text',
    **kwargs: str,
) -> str:
    """Format validation results in the requested format."""
    format_override = kwargs.pop('format', None)
    if kwargs:
        unexpected = ', '.join(sorted(kwargs))
        raise TypeError(f'Unexpected keyword arguments: {unexpected}')

    if format_override is not None:
        output_format = format_override

    if output_format not in ('text', 'json', 'csv'):
        raise ValueError(f"Invalid format '{output_format}'. Supported formats: text, json, csv")

    if output_format == 'json':
        formatter = JSONFormatter()
        return formatter.format_batch(results)

    if output_format == 'csv':
        formatter = CSVFormatter()
        lines = [formatter.get_header()]
        for result in results:
            lines.append(formatter.format_result(result))
        return '\n'.join(lines)

    formatter = TextFormatter()
    return formatter.format_batch(results)
