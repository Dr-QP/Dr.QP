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

"""Validator for SKILL.md content structure."""

import re
from typing import List

from . import SkillValidator, ValidationIssue


class StructureValidator(SkillValidator):
    """Validates required sections and structure in SKILL.md body."""
    
    REQUIRED_SECTIONS = [
        'when to use this skill',
        'prerequisites',
    ]
    
    RECOMMENDED_SECTIONS = [
        'troubleshooting',
        'step-by-step',
        'workflow',
    ]
    
    MAX_BODY_LINES = 500
    MIN_WORKFLOW_SECTIONS = 2
    
    def validate(self, skill_path: str, content: str, frontmatter: dict, body: str) -> List[ValidationIssue]:
        """Validate content structure."""
        issues = []
        
        if not body or not body.strip():
            issues.append(self.add_error("SKILL.md body is empty - content is required"))
            return issues
        
        body_lower = body.lower()
        
        # Check for required sections
        for section in self.REQUIRED_SECTIONS:
            if section not in body_lower:
                issues.append(self.add_error(
                    f"Missing required section: '{section.title()}'. "
                    f"This section is essential for skill documentation.",
                    section="structure"
                ))
        
        # Check for recommended sections (warnings)
        if self.show_warnings:
            for section in self.RECOMMENDED_SECTIONS:
                if section not in body_lower:
                    issues.append(self.add_warning(
                        f"Missing recommended section containing '{section}'. "
                        f"Consider adding this for better documentation.",
                        section="structure"
                    ))
        
        # Check for workflow sections (at least 2)
        workflow_count = self._count_workflow_sections(body)
        if workflow_count < self.MIN_WORKFLOW_SECTIONS:
            issues.append(self.add_warning(
                f"Found {workflow_count} workflow/step-by-step sections, recommend at least {self.MIN_WORKFLOW_SECTIONS} "
                f"for comprehensive guidance.",
                section="structure"
            ) if self.show_warnings else None)
        
        # Check for troubleshooting table
        if 'troubleshooting' in body_lower and not self._has_table(body):
            issues.append(self.add_warning(
                "Troubleshooting section found but no table detected. "
                "Consider using a table format for common issues and solutions.",
                section="structure"
            ) if self.show_warnings else None)
        
        # Check body length
        body_lines = len(body.split('\n'))
        if body_lines > self.MAX_BODY_LINES and self.show_warnings:
            issues.append(self.add_warning(
                f"Body content is {body_lines} lines (max {self.MAX_BODY_LINES} recommended). "
                f"Consider splitting large workflows into separate files in references/ folder.",
                section="structure"
            ))
        
        # Filter out None values
        issues = [i for i in issues if i is not None]
        
        return issues
    
    def _count_workflow_sections(self, body: str) -> int:
        """Count workflow/step-by-step sections."""
        # Look for numbered lists or sections with workflow-like headers
        workflow_patterns = [
            r'##\s+.*(?:workflow|step|procedure|process)',
            r'^\d+\.\s+',  # Numbered lists
            r'##\s+.*(?:how to|usage)',
        ]
        
        count = 0
        for pattern in workflow_patterns:
            matches = re.findall(pattern, body, re.MULTILINE | re.IGNORECASE)
            count += len(matches)
        
        return count
    
    def _has_table(self, body: str) -> bool:
        """Check if body contains a markdown table."""
        # Look for markdown table syntax
        table_pattern = r'\|.*\|.*\n\|[-:\s|]+\|'
        return bool(re.search(table_pattern, body))
