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

"""Base classes and utilities for skill validation."""

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional


class ValidationLevel(Enum):
    """Validation issue severity levels."""
    ERROR = "error"
    WARNING = "warning"
    INFO = "info"


@dataclass
class ValidationIssue:
    """Represents a validation issue found in a skill."""
    level: ValidationLevel
    message: str
    line_number: Optional[int] = None
    section: Optional[str] = None

    def __str__(self) -> str:
        """Format issue for display."""
        prefix = "✗" if self.level == ValidationLevel.ERROR else "⚠" if self.level == ValidationLevel.WARNING else "ℹ"
        location = f" (line {self.line_number})" if self.line_number else ""
        section_info = f" [{self.section}]" if self.section else ""
        return f"  {prefix} {self.message}{section_info}{location}"


@dataclass
class ValidationResult:
    """Result of validating a skill."""
    skill_path: str
    issues: List[ValidationIssue]
    
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
        status = "✓" if self.is_valid else "✗"
        return f"{status} {self.skill_path}"


class SkillValidator:
    """Base class for skill validators."""
    
    def __init__(self, show_warnings: bool = False):
        """Initialize validator.
        
        Args:
            show_warnings: Whether to include warnings in results
        """
        self.show_warnings = show_warnings
    
    def validate(self, skill_path: str, content: str, frontmatter: dict, body: str) -> List[ValidationIssue]:
        """Validate a skill file.
        
        Args:
            skill_path: Path to the skill file
            content: Full content of the skill file
            frontmatter: Parsed frontmatter dictionary
            body: Body content (after frontmatter)
            
        Returns:
            List of validation issues found
        """
        raise NotImplementedError("Subclasses must implement validate()")
    
    def add_error(self, message: str, line_number: Optional[int] = None, section: Optional[str] = None) -> ValidationIssue:
        """Create an error issue."""
        return ValidationIssue(ValidationLevel.ERROR, message, line_number, section)
    
    def add_warning(self, message: str, line_number: Optional[int] = None, section: Optional[str] = None) -> ValidationIssue:
        """Create a warning issue."""
        return ValidationIssue(ValidationLevel.WARNING, message, line_number, section)
    
    def add_info(self, message: str, line_number: Optional[int] = None, section: Optional[str] = None) -> ValidationIssue:
        """Create an info issue."""
        return ValidationIssue(ValidationLevel.INFO, message, line_number, section)
