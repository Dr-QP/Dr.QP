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

"""Validators for skill frontmatter and structure."""

from typing import List
import re

from validate_skills.types import ValidationIssue, ValidationLevel


class SkillFrontmatterValidator:
    """Validates skill frontmatter."""

    def validate(self, frontmatter: dict, show_warnings: bool = False) -> List[ValidationIssue]:
        """Validate frontmatter content.

        Args:
            frontmatter: Dictionary of frontmatter values
            show_warnings: Whether to include warnings

        Returns:
            List of validation issues
        """
        issues = []

        # Check required fields
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
            # Type check: name must be a string
            if not isinstance(name, str):
                issues.append(
                    ValidationIssue(
                        level=ValidationLevel.ERROR,
                        message=f"Name must be a string, got: {type(name).__name__}",
                        section='frontmatter',
                    )
                )
            else:
                # Validate name format
                if not re.match(r'^[a-z0-9]+(-[a-z0-9]+)*$', name):
                    issues.append(
                        ValidationIssue(
                            level=ValidationLevel.ERROR,
                            message=f"Name must be lowercase with hyphens (e.g., 'my-skill'), got: '{name}'",
                            section='frontmatter',
                        )
                    )
                # Check name length
                if len(name) > 64:
                    issues.append(
                        ValidationIssue(
                            level=ValidationLevel.ERROR,
                            message=f'Name exceeds maximum length of 64 characters',
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
            # Type check: description must be a string
            if not isinstance(description, str):
                issues.append(
                    ValidationIssue(
                        level=ValidationLevel.ERROR,
                        message=f"Description must be a string, got: {type(description).__name__}",
                        section='frontmatter',
                    )
                )
            else:
                # Check description length - minimum 10 chars, maximum 1024
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

                # Check for vague terms
                if show_warnings:
                    vague_terms = {'helpers', 'utilities', 'tools', 'functions', 'methods', 'stuff', 'things', 'misc', 'various', 'general'}
                    desc_lower = description.lower()
                    found_vague = [term for term in vague_terms if f' {term} ' in f' {desc_lower} ']
                    if found_vague:
                        issues.append(
                            ValidationIssue(
                                level=ValidationLevel.WARNING,
                                message=f"Description contains vague terms: {', '.join(found_vague)}. Be specific about capabilities.",
                                section='frontmatter',
                            )
                        )

        return issues


class SkillStructureValidator:
    """Validates skill markdown structure."""

    def validate(self, body: str, show_warnings: bool = False) -> List[ValidationIssue]:
        """Validate body content structure.

        Args:
            body: The markdown body content
            show_warnings: Whether to include warnings

        Returns:
            List of validation issues
        """
        issues = []
        body_lower = body.lower()

        # Check for required sections
        required_sections = {'# overview', '## when to use this skill'}
        missing = [s for s in required_sections if s not in body_lower]
        if missing:
            for section in missing:
                issues.append(
                    ValidationIssue(
                        level=ValidationLevel.ERROR,
                        message=f"Missing required section: '{section}'",
                        section='structure',
                    )
                )

        # Check for empty body
        if not body or not body.strip():
            issues.append(
                ValidationIssue(
                    level=ValidationLevel.ERROR,
                    message='Body content is empty',
                    section='structure',
                )
            )

        # Warn about empty sections if showing warnings
        if show_warnings:
            if '# overview' in body_lower:
                # Find the overview section and check if it's empty
                overview_idx = body_lower.find('# overview')
                next_section = body_lower.find('\n#', overview_idx + 1)
                if next_section == -1:
                    next_section = len(body)
                content = body[overview_idx + len('# overview'):next_section].strip()
                if not content or len(content) < 10:
                    issues.append(
                        ValidationIssue(
                            level=ValidationLevel.WARNING,
                            message='Overview section appears to be empty or very short',
                            section='structure',
                        )
                    )

        return issues
