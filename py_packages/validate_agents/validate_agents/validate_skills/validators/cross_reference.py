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

"""Validator for cross-references."""

from pathlib import Path
import re
from typing import List, Optional

from ..types import ValidationIssue, ValidationLevel


class CrossReferenceValidator:
    """Validates cross-references in skills."""

    def __init__(self, base_path: Optional[str] = None, show_warnings: bool = False):
        """Initialize the validator."""
        self.base_path = base_path
        self.show_warnings = show_warnings

    def validate(
        self,
        skill_path: str,
        metadata: dict,
        content: str,
        line_offset: int = 0,
    ) -> List[ValidationIssue]:
        """
        Validate cross-references.

        Args:
            skill_path: Path to the skill file
            metadata: Metadata dictionary (frontmatter)
            content: Content (body)
            line_offset: Number of file lines preceding the body content

        Returns:
            List of validation issues

        """
        issues = []

        # Extract all markdown links: [text](reference)
        link_pattern = r'\[([^\]]+)\]\(([^\)]+)\)'
        matches = re.finditer(link_pattern, content)

        for match in matches:
            text, reference = match.groups()
            # Skip external references
            if reference.startswith(('http://', 'https://', '#')):
                continue

            line_number, column_number = self._get_position(content, match.start())
            line_number += line_offset

            # Resolve path relative to base_path or skill directory
            if self.base_path:
                base = Path(self.base_path)
            else:
                base = Path(skill_path).parent

            # Try to find the referenced file
            ref_path = base / reference

            # Normalize path
            try:
                ref_path = ref_path.resolve()
            except (OSError, ValueError):
                # Path resolution failed
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

            # Check if file exists
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
    def _get_position(content: str, offset: int) -> tuple[int, int]:
        """Return 1-based line and column for a character offset."""
        line_number = content.count('\n', 0, offset) + 1
        line_start = content.rfind('\n', 0, offset)
        column_number = offset + 1 if line_start == -1 else offset - line_start
        return line_number, column_number
