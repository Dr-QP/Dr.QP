#!/usr/bin/env python3

"""Validator for skill name uniqueness."""

from typing import Dict, List, Optional

from ..types import ValidationIssue, ValidationLevel


class UniquenessValidator:
    """Validates skill name uniqueness."""

    def __init__(self, all_skills: Optional[Dict] = None):
        self.all_skills = all_skills or {}

    def validate(
        self, skill_path: str, metadata: dict, content: str = '', all_skills: Optional[Dict] = None
    ) -> List[ValidationIssue]:
        issues = []
        skills_to_check = all_skills or self.all_skills

        if 'name' not in metadata:
            return issues

        current_name = metadata.get('name')
        current_name_lower = (
            current_name.lower() if isinstance(current_name, str) else str(current_name).lower()
        )

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
