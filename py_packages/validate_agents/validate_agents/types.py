#!/usr/bin/env python3

"""Shared types for validation."""

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional


class ValidationLevel(Enum):
    """Severity levels for validation issues."""

    ERROR = 'error'
    WARNING = 'warning'
    INFO = 'info'


@dataclass
class ValidationIssue:
    """A single validation issue found in a file."""

    level: ValidationLevel
    message: str
    line_number: Optional[int] = None
    column_number: Optional[int] = None
    section: Optional[str] = None

    def __str__(self) -> str:
        """Format issue for display."""
        prefix = (
            '✗'
            if self.level == ValidationLevel.ERROR
            else '⚠'
            if self.level == ValidationLevel.WARNING
            else 'ℹ'
        )
        if self.line_number and self.column_number:
            location = f' (line {self.line_number}:{self.column_number})'
        elif self.line_number:
            location = f' (line {self.line_number})'
        else:
            location = ''
        section_info = f' [{self.section}]' if self.section else ''
        return f'  {prefix} {self.message}{section_info}{location}'


@dataclass
class ValidationResult:
    """Result of validating a single file."""

    skill_path: str
    issues: List[ValidationIssue] = field(default_factory=list)

    @property
    def is_valid(self) -> bool:
        """Check if validation passed (no errors)."""
        return not any(issue.level == ValidationLevel.ERROR for issue in self.issues)

    @property
    def has_warnings(self) -> bool:
        """Check if there are any warnings."""
        return any(issue.level == ValidationLevel.WARNING for issue in self.issues)

    def __str__(self) -> str:
        """Format result for display."""
        status = '✓' if self.is_valid else '✗'
        return f'{status} {self.skill_path}'
