#!/usr/bin/env python3

"""Validator for cross-references."""

from pathlib import Path
import re
from typing import List, Optional, Tuple

from ..types import ValidationIssue, ValidationLevel


class CrossReferenceValidator:
    """Validates cross-references in markdown files."""

    IGNORE_START_PATTERN = re.compile(
        r'<!--\s*validate_skills:\s*ignore-cross-reference-start\s*-->'
    )
    IGNORE_END_PATTERN = re.compile(r'<!--\s*validate_skills:\s*ignore-cross-reference-end\s*-->')

    def __init__(self, base_path: Optional[str] = None, show_warnings: bool = False):
        self.base_path = base_path
        self.show_warnings = show_warnings

    def validate(
        self,
        skill_path: str,
        metadata: dict,
        content: str,
        line_offset: int = 0,
    ) -> List[ValidationIssue]:
        issues = []
        ignored_ranges = self._build_ignored_ranges(content)
        link_pattern = r'\[([^\]]+)\]\(([^\)]+)\)'
        matches = re.finditer(link_pattern, content)

        for match in matches:
            _, reference = match.groups()

            if self._is_ignored_position(match.start(), ignored_ranges):
                continue

            if reference.startswith(('http://', 'https://', '#')):
                continue

            line_number, column_number = self._get_position(content, match.start())
            line_number += line_offset

            base = Path(self.base_path) if self.base_path else Path(skill_path).parent

            if reference.startswith('path/to/'):
                continue

            ref_path = self._resolve_reference(base, reference)

            try:
                ref_path = ref_path.resolve()
            except (OSError, ValueError):
                issues.append(
                    ValidationIssue(
                        level=ValidationLevel.WARNING,
                        message=f'Broken reference: {reference}',
                        line_number=line_number,
                        column_number=column_number,
                        section='cross_reference',
                    )
                )
                continue

            if not ref_path.exists():
                issues.append(
                    ValidationIssue(
                        level=ValidationLevel.ERROR,
                        message=f'Broken reference: {reference}',
                        line_number=line_number,
                        column_number=column_number,
                        section='cross_reference',
                    )
                )

        return issues

    @staticmethod
    def _resolve_reference(base: Path, reference: str) -> Path:
        if reference.startswith('/'):
            return (Path.cwd() / reference.lstrip('/')).resolve()
        return (base / reference).resolve()

    @staticmethod
    def _get_position(content: str, offset: int) -> tuple[int, int]:
        line_number = content.count('\n', 0, offset) + 1
        line_start = content.rfind('\n', 0, offset)
        column_number = offset + 1 if line_start == -1 else offset - line_start
        return line_number, column_number

    @classmethod
    def _build_ignored_ranges(cls, content: str) -> List[Tuple[int, int]]:
        boundaries = []
        for match in cls.IGNORE_START_PATTERN.finditer(content):
            boundaries.append((match.start(), 'start'))
        for match in cls.IGNORE_END_PATTERN.finditer(content):
            boundaries.append((match.start(), 'end'))
        boundaries.sort(key=lambda item: item[0])

        ranges: List[Tuple[int, int]] = []
        active_start: Optional[int] = None
        for position, boundary_type in boundaries:
            if boundary_type == 'start' and active_start is None:
                active_start = position
                continue
            if boundary_type == 'end' and active_start is not None:
                ranges.append((active_start, position))
                active_start = None

        if active_start is not None:
            ranges.append((active_start, len(content)))

        return ranges

    @staticmethod
    def _is_ignored_position(position: int, ranges: List[Tuple[int, int]]) -> bool:
        return any(start <= position < end for start, end in ranges)
