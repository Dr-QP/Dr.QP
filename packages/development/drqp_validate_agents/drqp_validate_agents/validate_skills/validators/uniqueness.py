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

"""Validator for skill name uniqueness."""

from typing import Dict, List, Optional

from drqp_validate_agents.validate_skills.types import ValidationIssue, ValidationLevel


class UniquenessValidator:
    """Validates skill name uniqueness."""

    def __init__(self, all_skills: Optional[Dict] = None):
        """Initialize the validator."""
        self.all_skills = all_skills or {}

    def validate(
        self, skill_path: str, metadata: dict, content: str = '', all_skills: Optional[Dict] = None
    ) -> List[ValidationIssue]:
        """
        Validate skill uniqueness.

        Args
        ----
            skill_path: Path to the skill file
            metadata: Metadata dictionary (frontmatter)
            content: Content string (for compatibility)
            all_skills: Override dict of all skills for comparison

        Returns
        -------
            List of validation issues

        """
        issues = []
        skills_to_check = all_skills or self.all_skills

        # Skip if no name in metadata
        if 'name' not in metadata:
            return issues

        current_name = metadata.get('name')
        current_name_lower = (
            current_name.lower() if isinstance(current_name, str) else str(current_name).lower()
        )

        # Check for duplicate names (case-insensitive)
        for other_path, other_data in skills_to_check.items():
            if other_path == skill_path:
                continue

            other_fm = other_data.get('frontmatter', {})
            other_name = other_fm.get('name')
            if other_name is None:
                continue

            other_name_lower = (
                other_name.lower() if isinstance(other_name, str) else str(other_name).lower()
            )

            if other_name_lower == current_name_lower:
                issues.append(
                    ValidationIssue(
                        level=ValidationLevel.ERROR,
                        message=f"Duplicate skill name '{current_name}' found in {other_path}",
                        section='uniqueness',
                    )
                )

        return issues
