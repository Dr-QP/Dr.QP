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

"""Validator for SKILL.md frontmatter."""

import re
from typing import List

from . import SkillValidator, ValidationIssue


class FrontmatterValidator(SkillValidator):
    """Validates YAML frontmatter in SKILL.md files."""
    
    MIN_DESCRIPTION_LENGTH = 100
    MAX_DESCRIPTION_LENGTH = 1024
    MAX_NAME_LENGTH = 64
    
    def validate(self, skill_path: str, content: str, frontmatter: dict, body: str) -> List[ValidationIssue]:
        """Validate frontmatter structure and content."""
        issues = []
        
        # Check required fields
        if not frontmatter:
            issues.append(self.add_error("No frontmatter found - YAML frontmatter is required"))
            return issues
        
        # Validate name field
        if 'name' not in frontmatter:
            issues.append(self.add_error("Missing required 'name' field in frontmatter", section="frontmatter"))
        else:
            issues.extend(self._validate_name(frontmatter['name']))
        
        # Validate description field
        if 'description' not in frontmatter:
            issues.append(self.add_error("Missing required 'description' field in frontmatter", section="frontmatter"))
        else:
            issues.extend(self._validate_description(frontmatter['description']))
        
        # Validate license field (optional but recommended)
        if 'license' not in frontmatter and self.show_warnings:
            issues.append(self.add_warning(
                "No 'license' field in frontmatter - consider adding license information",
                section="frontmatter"
            ))
        
        return issues
    
    def _validate_name(self, name: str) -> List[ValidationIssue]:
        """Validate the name field."""
        issues = []
        
        if not name:
            issues.append(self.add_error("'name' field is empty", section="frontmatter"))
            return issues
        
        # Check length
        if len(name) > self.MAX_NAME_LENGTH:
            issues.append(self.add_error(
                f"'name' exceeds maximum length of {self.MAX_NAME_LENGTH} characters (got {len(name)})",
                section="frontmatter"
            ))
        
        # Check format: lowercase with hyphens
        if not re.match(r'^[a-z0-9]+(-[a-z0-9]+)*$', name):
            issues.append(self.add_error(
                f"'name' must be lowercase with hyphens (e.g., 'my-skill-name'), got: '{name}'",
                section="frontmatter"
            ))
        
        return issues
    
    def _validate_description(self, description: str) -> List[ValidationIssue]:
        """Validate the description field."""
        issues = []
        
        if not description:
            issues.append(self.add_error("'description' field is empty", section="frontmatter"))
            return issues
        
        # Check length constraints
        desc_len = len(description)
        if desc_len < self.MIN_DESCRIPTION_LENGTH:
            issues.append(self.add_error(
                f"'description' is too short ({desc_len} chars). Minimum {self.MIN_DESCRIPTION_LENGTH} characters required for effective skill discovery",
                section="frontmatter"
            ))
        
        if desc_len > self.MAX_DESCRIPTION_LENGTH:
            issues.append(self.add_error(
                f"'description' exceeds maximum length of {self.MAX_DESCRIPTION_LENGTH} characters (got {desc_len})",
                section="frontmatter"
            ))
        
        return issues
