#!/usr/bin/env python3

"""Validation rules for .prompt.md files."""

from __future__ import annotations

from pathlib import Path
import re
from typing import List

from ..types import ValidationIssue, ValidationLevel

FILE_REFERENCE_PATTERN = re.compile(r'#file:([^\s]+)')


def validate_prompt_frontmatter(frontmatter: dict) -> List[ValidationIssue]:
    issues: List[ValidationIssue] = []
    agent = frontmatter.get('agent')
    if not isinstance(agent, str) or not agent.strip():
        issues.append(
            ValidationIssue(
                level=ValidationLevel.ERROR,
                message='Missing required field in frontmatter: agent',
                section='frontmatter',
            )
        )

    model = frontmatter.get('model')
    if model is not None and not isinstance(model, str):
        issues.append(
            ValidationIssue(
                level=ValidationLevel.ERROR,
                message="Field 'model' must be a string",
                section='frontmatter',
            )
        )

    return issues


def validate_prompt_body(body: str) -> List[ValidationIssue]:
    if body.strip():
        return []

    return [
        ValidationIssue(
            level=ValidationLevel.ERROR,
            message='Prompt body is empty',
            section='structure',
        )
    ]


def validate_prompt_references(
    file_path: str, body: str, line_offset: int
) -> List[ValidationIssue]:
    issues: List[ValidationIssue] = []
    base_dir = Path(file_path).parent
    for match in FILE_REFERENCE_PATTERN.finditer(body):
        reference = match.group(1).rstrip(').,;')
        resolved = (base_dir / reference).resolve(strict=False)
        if resolved.exists():
            continue

        line_number, column_number = _position_from_offset(body, match.start())
        issues.append(
            ValidationIssue(
                level=ValidationLevel.ERROR,
                message=f'Broken reference: {reference}',
                line_number=line_number + line_offset,
                column_number=column_number,
                section='cross_reference',
            )
        )

    return issues


def _position_from_offset(content: str, offset: int) -> tuple[int, int]:
    line_number = content.count('\n', 0, offset) + 1
    line_start = content.rfind('\n', 0, offset)
    column_number = offset + 1 if line_start == -1 else offset - line_start
    return line_number, column_number
