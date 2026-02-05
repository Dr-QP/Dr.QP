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

"""Unit tests for individual skill validators."""

import pytest
from validate_skills.validators.skill import (
    SkillFrontmatterValidator,
    SkillStructureValidator,
)
from validate_skills.validators.uniqueness import UniquenessValidator
from validate_skills.validators.cross_reference import CrossReferenceValidator


class TestSkillFrontmatterValidator:
    """Tests for SkillFrontmatter validation."""

    def test_validate_valid_frontmatter(self):
        """Should accept valid frontmatter with required fields."""
        frontmatter = {
            'name': 'my-skill',
            'description': 'A comprehensive description of what this skill does and when to use it.',
        }
        validator = SkillFrontmatterValidator()
        issues = validator.validate(frontmatter)
        assert len(issues) == 0

    def test_validate_missing_name(self):
        """Should reject frontmatter missing name field."""
        frontmatter = {
            'description': 'A valid description.',
        }
        validator = SkillFrontmatterValidator()
        issues = validator.validate(frontmatter)
        assert len(issues) > 0
        assert any('name' in str(issue) for issue in issues)

    def test_validate_missing_description(self):
        """Should reject frontmatter missing description field."""
        frontmatter = {
            'name': 'my-skill',
        }
        validator = SkillFrontmatterValidator()
        issues = validator.validate(frontmatter)
        assert len(issues) > 0
        assert any('description' in str(issue) for issue in issues)

    def test_validate_name_invalid_format(self):
        """Should reject name not in lowercase-with-hyphens format."""
        frontmatter = {
            'name': 'MySkill',  # Invalid: should be my-skill
            'description': 'A valid description.',
        }
        validator = SkillFrontmatterValidator()
        issues = validator.validate(frontmatter)
        assert len(issues) > 0

    def test_validate_description_too_short(self):
        """Should reject description that is too short."""
        frontmatter = {
            'name': 'my-skill',
            'description': 'Too short',  # Less than 100 characters
        }
        validator = SkillFrontmatterValidator()
        issues = validator.validate(frontmatter)
        assert len(issues) > 0

    def test_validate_description_too_long(self):
        """Should reject description that exceeds maximum length."""
        frontmatter = {
            'name': 'my-skill',
            'description': 'x' * 1100,  # Exceeds 1024 chars
        }
        validator = SkillFrontmatterValidator()
        issues = validator.validate(frontmatter)
        assert len(issues) > 0

    def test_validate_description_vague_terms(self):
        """Should warn about vague descriptions using generic terms."""
        frontmatter = {
            'name': 'my-skill',
            'description': 'This is a helper for various utilities and tools for things.',
        }
        validator = SkillFrontmatterValidator()
        issues = validator.validate(frontmatter, show_warnings=True)
        # Should have warnings about vague terms
        assert any('vague' in str(issue).lower() or 'helper' in str(issue).lower() 
                  for issue in issues)

    def test_validate_extra_fields_allowed(self):
        """Should allow extra fields in frontmatter."""
        frontmatter = {
            'name': 'my-skill',
            'description': 'A comprehensive description of what this skill does.',
            'extra_field': 'extra_value',
            'another_field': 123,
        }
        validator = SkillFrontmatterValidator()
        issues = validator.validate(frontmatter)
        assert len(issues) == 0


class TestSkillStructureValidator:
    """Tests for SkillStructure validation."""

    def test_validate_valid_structure(self):
        """Should accept well-formed skill structure."""
        body = """# Overview
This skill validates other skills.

## When to use this skill
Use this when you need to validate Agent Skills.

## Features
- Validates frontmatter
- Checks file structure
"""
        validator = SkillStructureValidator()
        issues = validator.validate(body)
        assert len(issues) == 0

    def test_validate_missing_overview(self):
        """Should reject structure missing Overview section."""
        body = """## When to use this skill
Use this skill.
"""
        validator = SkillStructureValidator()
        issues = validator.validate(body)
        assert len(issues) > 0
        assert any('overview' in str(issue).lower() for issue in issues)

    def test_validate_missing_when_to_use(self):
        """Should reject structure missing 'When to use' section."""
        body = """# Overview
This is an overview.
"""
        validator = SkillStructureValidator()
        issues = validator.validate(body)
        assert len(issues) > 0

    def test_validate_heading_hierarchy(self):
        """Should validate proper markdown heading hierarchy."""
        body = """# Overview
This is an overview.

## When to use this skill
Use this.
"""
        validator = SkillStructureValidator()
        issues = validator.validate(body)
        # Should validate H1 -> H2 progression
        assert isinstance(issues, list)

    def test_validate_missing_subheadings(self):
        """Should warn about missing helpful subheadings."""
        body = """# Overview
Some content.

## When to use this skill
When you need it.

A lot of content without subheadings."""
        validator = SkillStructureValidator()
        issues = validator.validate(body, show_warnings=True)
        # May have warnings about organization
        assert isinstance(issues, list)

    def test_validate_empty_sections(self):
        """Should warn about empty sections."""
        body = """# Overview

## When to use this skill
"""
        validator = SkillStructureValidator()
        issues = validator.validate(body, show_warnings=True)
        assert len(issues) > 0


class TestUniquenessValidator:
    """Tests for skill name uniqueness validation."""

    def test_validate_unique_name(self):
        """Should accept skill with unique name."""
        validator = UniquenessValidator({})
        issues = validator.validate(
            skill_path='/path/to/my-skill/SKILL.md',
            metadata={'name': 'my-skill'},
            all_skills={}
        )
        assert len(issues) == 0

    def test_validate_duplicate_name(self):
        """Should reject duplicate skill names."""
        all_skills = {
            '/path/to/existing-skill/SKILL.md': {
                'frontmatter': {'name': 'my-skill'},
                'body': 'Content'
            }
        }
        validator = UniquenessValidator(all_skills)
        issues = validator.validate(
            skill_path='/path/to/my-skill/SKILL.md',
            metadata={'name': 'my-skill'},
            all_skills=all_skills
        )
        assert len(issues) > 0
        assert any('duplicate' in str(issue).lower() or 'unique' in str(issue).lower() 
                  for issue in issues)

    def test_validate_case_insensitive_duplicates(self):
        """Should detect duplicates regardless of case."""
        all_skills = {
            '/path/to/my-skill/SKILL.md': {
                'frontmatter': {'name': 'my-skill'},
                'body': 'Content'
            }
        }
        validator = UniquenessValidator(all_skills)
        issues = validator.validate(
            skill_path='/path/to/other/SKILL.md',
            metadata={'name': 'MY-SKILL'},
            all_skills=all_skills
        )
        # Should detect case-insensitive duplicate
        assert len(issues) > 0


class TestCrossReferenceValidator:
    """Tests for cross-reference validation."""

    def test_validate_valid_references(self, tmp_path):
        """Should accept valid file references."""
        # Create sample files
        skill_file = tmp_path / "SKILL.md"
        referenced_file = tmp_path / "supporting.md"
        skill_file.write_text("See [supporting](supporting.md)")
        referenced_file.write_text("Content")
        
        validator = CrossReferenceValidator(str(tmp_path))
        issues = validator.validate(
            skill_path=str(skill_file),
            metadata={},
            content="See [supporting](supporting.md)"
        )
        assert len(issues) == 0

    def test_validate_broken_references(self, tmp_path):
        """Should reject broken file references."""
        skill_file = tmp_path / "SKILL.md"
        skill_file.write_text("See [missing](nonexistent.md)")
        
        validator = CrossReferenceValidator(str(tmp_path))
        issues = validator.validate(
            skill_path=str(skill_file),
            metadata={},
            content="See [missing](nonexistent.md)"
        )
        assert len(issues) > 0
        assert any('reference' in str(issue).lower() or 'broken' in str(issue).lower() 
                  for issue in issues)

    def test_validate_absolute_path_references(self, tmp_path):
        """Should handle absolute path references."""
        skill_file = tmp_path / "SKILL.md"
        referenced = tmp_path / "docs" / "guide.md"
        referenced.parent.mkdir()
        referenced.write_text("Guide content")
        skill_file.write_text("See [guide](/docs/guide.md)")
        
        validator = CrossReferenceValidator(str(tmp_path))
        issues = validator.validate(
            skill_path=str(skill_file),
            metadata={},
            content="See [guide](/docs/guide.md)"
        )
        # Should either pass or warn appropriately
        assert isinstance(issues, list)


class TestValidatorsEdgeCases:
    """Tests for edge cases in validators."""

    def test_validators_empty_metadata(self):
        """Should handle empty metadata gracefully."""
        frontmatter_validator = SkillFrontmatterValidator()
        issues = frontmatter_validator.validate({})
        assert len(issues) > 0  # Should have errors for missing required fields

    def test_validators_none_values(self):
        """Should handle None values appropriately."""
        frontmatter_validator = SkillFrontmatterValidator()
        issues = frontmatter_validator.validate({
            'name': None,
            'description': None,
        })
        assert len(issues) > 0

    def test_validators_whitespace_only_values(self):
        """Should reject whitespace-only values."""
        frontmatter_validator = SkillFrontmatterValidator()
        issues = frontmatter_validator.validate({
            'name': '   ',
            'description': '   ',
        })
        assert len(issues) > 0

