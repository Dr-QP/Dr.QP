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

"""Tests for skill validation."""

import os
import sys
import unittest

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from skill_validators import ValidationLevel
from skill_validators.description_validator import DescriptionValidator
from skill_validators.frontmatter_validator import FrontmatterValidator
from skill_validators.structure_validator import StructureValidator
from skill_validators.uniqueness_validator import UniquenessValidator


class TestFrontmatterValidator(unittest.TestCase):
    """Tests for frontmatter validation."""
    
    def setUp(self):
        self.validator = FrontmatterValidator(show_warnings=True)
    
    def test_valid_frontmatter(self):
        """Test valid frontmatter passes validation."""
        frontmatter = {
            'name': 'my-skill',
            'description': 'A' * 150,  # Valid length
            'license': 'MIT'
        }
        issues = self.validator.validate('test.md', '', frontmatter, '')
        errors = [i for i in issues if i.level == ValidationLevel.ERROR]
        self.assertEqual(len(errors), 0)
    
    def test_missing_name(self):
        """Test missing name field is detected."""
        frontmatter = {'description': 'A' * 150}
        issues = self.validator.validate('test.md', '', frontmatter, '')
        errors = [i for i in issues if i.level == ValidationLevel.ERROR]
        self.assertTrue(any('name' in i.message.lower() for i in errors))
    
    def test_missing_description(self):
        """Test missing description field is detected."""
        frontmatter = {'name': 'my-skill'}
        issues = self.validator.validate('test.md', '', frontmatter, '')
        errors = [i for i in issues if i.level == ValidationLevel.ERROR]
        self.assertTrue(any('description' in i.message.lower() for i in errors))
    
    def test_invalid_name_format(self):
        """Test invalid name format is detected."""
        frontmatter = {
            'name': 'Invalid_Name',  # Underscores not allowed
            'description': 'A' * 150
        }
        issues = self.validator.validate('test.md', '', frontmatter, '')
        errors = [i for i in issues if i.level == ValidationLevel.ERROR]
        self.assertTrue(any('lowercase' in i.message.lower() for i in errors))
    
    def test_description_too_short(self):
        """Test description too short is detected."""
        frontmatter = {
            'name': 'my-skill',
            'description': 'Too short'
        }
        issues = self.validator.validate('test.md', '', frontmatter, '')
        errors = [i for i in issues if i.level == ValidationLevel.ERROR]
        self.assertTrue(any('too short' in i.message.lower() for i in errors))
    
    def test_description_too_long(self):
        """Test description too long is detected."""
        frontmatter = {
            'name': 'my-skill',
            'description': 'A' * 1100  # Over 1024 chars
        }
        issues = self.validator.validate('test.md', '', frontmatter, '')
        errors = [i for i in issues if i.level == ValidationLevel.ERROR]
        self.assertTrue(any('exceeds maximum' in i.message.lower() for i in errors))


class TestDescriptionValidator(unittest.TestCase):
    """Tests for description pattern validation."""
    
    def setUp(self):
        self.validator = DescriptionValidator(show_warnings=True)
    
    def test_valid_description(self):
        """Test valid description with WHAT/WHEN/KEYWORDS."""
        frontmatter = {
            'description': 'Toolkit for testing local web applications using Playwright. '
                          'Use when asked to verify frontend functionality, debug UI behavior, '
                          'capture screenshots, check regressions, or view console logs. '
                          'Supports Chrome, Firefox, and WebKit browsers.'
        }
        issues = self.validator.validate('test.md', '', frontmatter, '')
        errors = [i for i in issues if i.level == ValidationLevel.ERROR]
        self.assertEqual(len(errors), 0)
    
    def test_vague_terms_detected(self):
        """Test vague terms are detected."""
        frontmatter = {
            'description': 'Some helpers and utilities for various things when you need general stuff'
        }
        issues = self.validator.validate('test.md', '', frontmatter, '')
        errors = [i for i in issues if i.level == ValidationLevel.ERROR]
        self.assertTrue(any('vague' in i.message.lower() for i in errors))
    
    def test_missing_when_guidance(self):
        """Test missing WHEN guidance is detected."""
        frontmatter = {
            'description': 'Playwright browser automation toolkit that handles screenshots '
                          'and console logging with multiple browser support.'
        }
        issues = self.validator.validate('test.md', '', frontmatter, '')
        errors = [i for i in issues if i.level == ValidationLevel.ERROR]
        self.assertTrue(any('when' in i.message.lower() for i in errors))
    
    def test_too_few_keywords(self):
        """Test too few keywords is detected."""
        frontmatter = {
            'description': 'Test stuff when asked'
        }
        issues = self.validator.validate('test.md', '', frontmatter, '')
        errors = [i for i in issues if i.level == ValidationLevel.ERROR]
        self.assertTrue(any('keyword' in i.message.lower() for i in errors))


class TestStructureValidator(unittest.TestCase):
    """Tests for structure validation."""
    
    def setUp(self):
        self.validator = StructureValidator(show_warnings=True)
    
    def test_valid_structure(self):
        """Test valid structure passes."""
        body = """
# Title

## When to Use This Skill

Use this when...

## Prerequisites

- Requirement 1
- Requirement 2

## Workflow 1

1. Step 1
2. Step 2

## Workflow 2

1. Step 1
2. Step 2

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Problem | Fix |
"""
        issues = self.validator.validate('test.md', '', {}, body)
        errors = [i for i in issues if i.level == ValidationLevel.ERROR]
        self.assertEqual(len(errors), 0)
    
    def test_missing_when_to_use(self):
        """Test missing 'When to Use This Skill' section."""
        body = """
# Title

## Prerequisites

- Requirement 1
"""
        issues = self.validator.validate('test.md', '', {}, body)
        errors = [i for i in issues if i.level == ValidationLevel.ERROR]
        self.assertTrue(any('when to use' in i.message.lower() for i in errors))
    
    def test_missing_prerequisites(self):
        """Test missing 'Prerequisites' section."""
        body = """
# Title

## When to Use This Skill

Use this when...
"""
        issues = self.validator.validate('test.md', '', {}, body)
        errors = [i for i in issues if i.level == ValidationLevel.ERROR]
        self.assertTrue(any('prerequisites' in i.message.lower() for i in errors))


class TestUniquenessValidator(unittest.TestCase):
    """Tests for uniqueness validation."""
    
    def setUp(self):
        self.validator = UniquenessValidator(show_warnings=True)
    
    def test_no_duplicates(self):
        """Test no duplicate names."""
        all_skills = {
            'skill1.md': {'frontmatter': {'name': 'skill-one'}, 'body': ''},
            'skill2.md': {'frontmatter': {'name': 'skill-two'}, 'body': ''},
        }
        validator = UniquenessValidator(show_warnings=True, all_skills=all_skills)
        frontmatter = {'name': 'skill-three', 'description': 'test'}
        issues = validator.validate('skill3.md', '', frontmatter, '')
        errors = [i for i in issues if i.level == ValidationLevel.ERROR]
        self.assertEqual(len(errors), 0)
    
    def test_duplicate_name_detected(self):
        """Test duplicate name is detected."""
        all_skills = {
            'skill1.md': {'frontmatter': {'name': 'duplicate-skill'}, 'body': ''},
        }
        validator = UniquenessValidator(show_warnings=True, all_skills=all_skills)
        frontmatter = {'name': 'duplicate-skill', 'description': 'test'}
        issues = validator.validate('skill2.md', '', frontmatter, '')
        errors = [i for i in issues if i.level == ValidationLevel.ERROR]
        self.assertTrue(any('duplicate' in i.message.lower() for i in errors))


class TestEndToEnd(unittest.TestCase):
    """End-to-end tests with example skills."""
    
    def test_good_skill_passes(self):
        """Test that good example skill passes validation."""
        fixture_path = os.path.join(
            os.path.dirname(__file__),
            'fixtures',
            'good-skill',
            'SKILL.md'
        )
        
        if not os.path.exists(fixture_path):
            self.skipTest(f"Good skill fixture not found: {fixture_path}")
        
        with open(fixture_path, 'r') as f:
            content = f.read()
        
        # Parse frontmatter manually for test
        import re
        match = re.match(r'^---\s*\n(.*?)\n---\s*\n(.*)$', content, re.DOTALL)
        self.assertIsNotNone(match, "Failed to parse frontmatter")
        
        import yaml
        frontmatter = yaml.safe_load(match.group(1))
        body = match.group(2)
        
        # Run all validators
        validators = [
            FrontmatterValidator(show_warnings=False),
            DescriptionValidator(show_warnings=False),
            StructureValidator(show_warnings=False),
        ]
        
        all_errors = []
        for validator in validators:
            issues = validator.validate(fixture_path, content, frontmatter, body)
            errors = [i for i in issues if i.level == ValidationLevel.ERROR]
            all_errors.extend(errors)
        
        self.assertEqual(len(all_errors), 0, f"Good skill should have no errors: {all_errors}")
    
    def test_bad_skill_fails(self):
        """Test that bad example skill fails validation."""
        fixture_path = os.path.join(
            os.path.dirname(__file__),
            'fixtures',
            'bad-skill',
            'SKILL.md'
        )
        
        if not os.path.exists(fixture_path):
            self.skipTest(f"Bad skill fixture not found: {fixture_path}")
        
        with open(fixture_path, 'r') as f:
            content = f.read()
        
        # Parse frontmatter manually for test
        import re
        match = re.match(r'^---\s*\n(.*?)\n---\s*\n(.*)$', content, re.DOTALL)
        self.assertIsNotNone(match, "Failed to parse frontmatter")
        
        import yaml
        frontmatter = yaml.safe_load(match.group(1))
        body = match.group(2)
        
        # Run all validators
        validators = [
            FrontmatterValidator(show_warnings=False),
            DescriptionValidator(show_warnings=False),
            StructureValidator(show_warnings=False),
        ]
        
        all_errors = []
        for validator in validators:
            issues = validator.validate(fixture_path, content, frontmatter, body)
            errors = [i for i in issues if i.level == ValidationLevel.ERROR]
            all_errors.extend(errors)
        
        self.assertGreater(len(all_errors), 0, "Bad skill should have errors")


if __name__ == '__main__':
    unittest.main()
