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

"""Pydantic models for skill validation."""

import re
from typing import ClassVar, Optional

from pydantic import BaseModel, Field, field_validator, ConfigDict


class SkillFrontmatter(BaseModel):
    """Pydantic model for SKILL.md frontmatter validation."""

    model_config = ConfigDict(extra='allow')  # Allow extra fields

    name: str = Field(
        ..., min_length=1, max_length=64, description='Skill name in lowercase-with-hyphens format'
    )
    description: str = Field(
        ..., min_length=100, max_length=1024, description='Detailed skill description'
    )
    license: Optional[str] = Field(None, description='License information (optional)')

    # Vague terms that indicate poor descriptions
    VAGUE_TERMS: ClassVar[set[str]] = {
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

    # Trigger phrases that indicate WHEN guidance
    WHEN_INDICATORS: ClassVar[set[str]] = {
        'use when',
        'when asked',
        'when you need',
        'when working',
        'when debugging',
        'when testing',
        'when building',
        'when creating',
        'for',
        'to verify',
        'to debug',
        'to test',
        'to build',
        'to analyze',
        'supports',
        'includes',
        'provides',
    }

    # Overly broad keywords
    BROAD_KEYWORDS: ClassVar[set[str]] = {
        'file',
        'code',
        'test',
        'build',
        'run',
        'help',
        'make',
        'create',
        'write',
        'read',
        'update',
    }

    MIN_UNIQUE_KEYWORDS: ClassVar[int] = 5

    @field_validator('name')
    @classmethod
    def validate_name_format(cls, v: str) -> str:
        """Validate name follows lowercase-with-hyphens format."""
        if not re.match(r'^[a-z0-9]+(-[a-z0-9]+)*$', v):
            raise ValueError(
                f"'name' must be lowercase with hyphens (e.g., 'my-skill-name'), got: '{v}'"
            )
        return v

    @field_validator('description')
    @classmethod
    def validate_description_content(cls, v: str) -> str:
        """Validate description follows WHAT/WHEN/KEYWORDS pattern."""
        if not v:
            raise ValueError("'description' field is empty")

        desc_lower = v.lower()

        # Check for vague terms
        found_vague = [term for term in cls.VAGUE_TERMS if term in desc_lower.split()]
        if found_vague:
            raise ValueError(
                f'Description contains vague terms that hinder discovery: {", ".join(found_vague)}. '
                f'Be specific about capabilities and use cases.'
            )

        # Check for WHEN indicators (triggers/scenarios)
        has_when = any(indicator in desc_lower for indicator in cls.WHEN_INDICATORS)
        if not has_when:
            raise ValueError(
                "Description missing WHEN guidance. Include specific triggers like 'Use when asked to...', "
                "'for debugging...', 'to verify...', etc."
            )

        # Extract and check keywords
        keywords = cls._extract_keywords(v)
        if len(keywords) < cls.MIN_UNIQUE_KEYWORDS:
            raise ValueError(
                f'Description has too few unique keywords ({len(keywords)} found, minimum {cls.MIN_UNIQUE_KEYWORDS} required). '
                f'Add specific technical terms, actions, or domain concepts.'
            )

        return v

    @classmethod
    def _extract_keywords(cls, text: str) -> set:
        """Extract unique keywords from text."""
        # Simple keyword extraction: split on whitespace and punctuation
        words = re.findall(r'\b[a-z]+\b', text.lower())
        # Filter out common stop words
        stop_words = {
            'the',
            'a',
            'an',
            'and',
            'or',
            'but',
            'in',
            'on',
            'at',
            'to',
            'for',
            'of',
            'with',
            'by',
            'from',
            'as',
            'is',
            'are',
            'was',
            'were',
            'be',
            'been',
            'have',
            'has',
            'had',
            'do',
            'does',
            'did',
            'will',
            'would',
            'should',
            'could',
            'may',
            'might',
            'must',
            'can',
            'this',
            'that',
            'these',
            'those',
            'it',
            'its',
        }
        keywords = {w for w in words if w not in stop_words and len(w) > 2}
        return keywords


class SkillStructure(BaseModel):
    """Pydantic model for SKILL.md structure validation."""

    model_config = ConfigDict(arbitrary_types_allowed=True)

    body: str = Field(..., min_length=1, description='Skill body content')

    REQUIRED_SECTIONS: ClassVar[list[str]] = [
        'when to use this skill',
        'prerequisites',
    ]

    RECOMMENDED_SECTIONS: ClassVar[list[str]] = [
        'troubleshooting',
        'step-by-step',
        'workflow',
    ]

    MAX_BODY_LINES: ClassVar[int] = 500

    @field_validator('body')
    @classmethod
    def validate_body_content(cls, v: str) -> str:
        """Validate body structure and required sections."""
        if not v or not v.strip():
            raise ValueError('SKILL.md body is empty - content is required')

        body_lower = v.lower()

        # Check for required sections
        missing_required = [
            section for section in cls.REQUIRED_SECTIONS if section not in body_lower
        ]

        if missing_required:
            sections_str = "', '".join(s.title() for s in missing_required)
            raise ValueError(
                f"Missing required section(s): '{sections_str}'. "
                f'These sections are essential for skill documentation.'
            )

        return v

    def get_warnings(self) -> list[str]:
        """Get non-critical warnings about the structure."""
        warnings = []
        body_lower = self.body.lower()

        # Check for recommended sections
        for section in self.RECOMMENDED_SECTIONS:
            if section not in body_lower:
                warnings.append(
                    f"Missing recommended section containing '{section}'. "
                    f'Consider adding this for better documentation.'
                )

        # Check body length
        body_lines = len(self.body.split('\n'))
        if body_lines > self.MAX_BODY_LINES:
            warnings.append(
                f'Body content is {body_lines} lines (max {self.MAX_BODY_LINES} recommended). '
                f'Consider splitting large workflows into separate files in references/ folder.'
            )

        return warnings
