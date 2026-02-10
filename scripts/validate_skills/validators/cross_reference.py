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

from validate_skills.types import ValidationIssue, ValidationLevel


class CrossReferenceValidator:
    """Validates cross-references in skills."""

    def __init__(self, base_path: Optional[str] = None, show_warnings: bool = False):
        """
        Initialize the validator.

        Args:
        -----
            base_path: Base path for resolving relative references.
            show_warnings: Whether to include warnings.

        """
        self.base_path = base_path
        self.show_warnings = show_warnings

    def validate(self, skill_path: str, metadata: dict, content: str) -> List[ValidationIssue]:
        """
        Validate cross-references.

        Args
        ----
            skill_path: Path to the skill file
            metadata: Metadata dictionary (frontmatter)
            content: Content (body)

        Returns
        -------
            List of validation issues

        """
        issues = []

        # Extract all markdown links: [text](reference)
        link_pattern = r'\[([^\]]+)\]\(([^\)]+)\)'
        matches = re.findall(link_pattern, content)

        for text, reference in matches:
            # Skip external references
            if reference.startswith(('http://', 'https://', '#')):
                continue

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
                        section='cross_reference',
                    )
                )

        return issues
