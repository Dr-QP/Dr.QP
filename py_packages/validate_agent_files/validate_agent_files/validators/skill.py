#!/usr/bin/env python3

"""Validators for skill frontmatter and structure."""

import re
from typing import List

from ..types import ValidationIssue, ValidationLevel


class SkillFrontmatterValidator:
    """Validates skill frontmatter."""

    def validate(self, frontmatter: dict, show_warnings: bool = False) -> List[ValidationIssue]:
        issues = []

        if 'name' not in frontmatter:
            issues.append(
                ValidationIssue(
                    level=ValidationLevel.ERROR,
                    message="Missing required field: 'name'",
                    section='frontmatter',
                )
            )
        else:
            name = frontmatter['name']
            if not isinstance(name, str):
                issues.append(
                    ValidationIssue(
                        level=ValidationLevel.ERROR,
                        message=f'Name must be a string, got: {type(name).__name__}',
                        section='frontmatter',
                    )
                )
            else:
                if not re.match(r'^[a-z0-9]+(-[a-z0-9]+)*$', name):
                    issues.append(
                        ValidationIssue(
                            level=ValidationLevel.ERROR,
                            message=(
                                f"Name must be lowercase with hyphens (e.g., 'my-skill'), got: '{name}'"
                            ),
                            section='frontmatter',
                        )
                    )
                if len(name) > 64:
                    issues.append(
                        ValidationIssue(
                            level=ValidationLevel.ERROR,
                            message='Name exceeds maximum length of 64 characters',
                            section='frontmatter',
                        )
                    )

        if 'description' not in frontmatter:
            issues.append(
                ValidationIssue(
                    level=ValidationLevel.ERROR,
                    message="Missing required field: 'description'",
                    section='frontmatter',
                )
            )
        else:
            description = frontmatter['description']
            if not isinstance(description, str):
                issues.append(
                    ValidationIssue(
                        level=ValidationLevel.ERROR,
                        message=f'Description must be a string, got: {type(description).__name__}',
                        section='frontmatter',
                    )
                )
            else:
                if len(description) < 10:
                    issues.append(
                        ValidationIssue(
                            level=ValidationLevel.ERROR,
                            message='Description must be at least 10 characters',
                            section='frontmatter',
                        )
                    )
                if len(description) > 1024:
                    issues.append(
                        ValidationIssue(
                            level=ValidationLevel.ERROR,
                            message='Description exceeds maximum length of 1024 characters',
                            section='frontmatter',
                        )
                    )

                if show_warnings:
                    vague_terms = {
                        'helpers',
                        'utilities',
                        'tools',
                        'functions',
                        'methods',
                        'stuff',
                        'things',
                        'misc',
                        'various',
                        'general',
                    }
                    desc_lower = description.lower()
                    found_vague = [
                        term for term in vague_terms if f' {term} ' in f' {desc_lower} '
                    ]
                    if found_vague:
                        issues.append(
                            ValidationIssue(
                                level=ValidationLevel.WARNING,
                                message=(
                                    'Description contains vague terms: '
                                    f"{', '.join(found_vague)}. Be specific about capabilities."
                                ),
                                section='frontmatter',
                            )
                        )

        return issues


class SkillStructureValidator:
    """Validates skill markdown structure."""

    def validate(self, body: str, show_warnings: bool = False) -> List[ValidationIssue]:
        issues = []

        if not body or not body.strip():
            issues.append(
                ValidationIssue(
                    level=ValidationLevel.ERROR,
                    message='Body content is empty',
                    section='structure',
                )
            )
            return issues

        header_matches = list(re.finditer(r'^#\s+.+$', body, flags=re.MULTILINE))
        if not header_matches:
            issues.append(
                ValidationIssue(
                    level=ValidationLevel.ERROR,
                    message="Missing required top-level header (e.g., '# Title')",
                    section='structure',
                )
            )
            return issues

        if show_warnings:
            first_header = header_matches[0]
            header_line = first_header.group(0)
            start_index = first_header.end()
            next_header = re.search(r'^#{1,6}\s+.+$', body[start_index:], flags=re.MULTILINE)
            end_index = start_index + next_header.start() if next_header else len(body)
            content = body[start_index:end_index].strip()
            if not content or len(content) < 10:
                issues.append(
                    ValidationIssue(
                        level=ValidationLevel.WARNING,
                        message=(
                            f"Top-level section '{header_line.strip()}' appears to be empty or very short"
                        ),
                        section='structure',
                    )
                )

        return issues
