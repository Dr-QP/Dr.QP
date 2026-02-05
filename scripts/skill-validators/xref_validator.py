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

"""Validator for cross-references (README, AGENTS.md, internal links)."""

import os
import re
from pathlib import Path
from typing import List

from . import SkillValidator, ValidationIssue


class CrossReferenceValidator(SkillValidator):
    """Validates cross-references to and from skill files."""
    
    def __init__(self, show_warnings: bool = False, repo_root: str = None):
        """Initialize cross-reference validator.
        
        Args:
            show_warnings: Whether to include warnings
            repo_root: Root directory of the repository
        """
        super().__init__(show_warnings)
        self.repo_root = repo_root or os.getcwd()
    
    def validate(self, skill_path: str, content: str, frontmatter: dict, body: str) -> List[ValidationIssue]:
        """Validate cross-references."""
        issues = []
        
        if not frontmatter or 'name' not in frontmatter:
            return issues  # Handled by frontmatter validator
        
        skill_name = frontmatter.get('name')
        skill_dir = os.path.dirname(skill_path)
        
        # Check if skill is listed in README.md
        issues.extend(self._check_readme_reference(skill_path, skill_name))
        
        # Check internal links in the skill
        issues.extend(self._check_internal_links(skill_path, body, skill_dir))
        
        return issues
    
    def _check_readme_reference(self, skill_path: str, skill_name: str) -> List[ValidationIssue]:
        """Check if skill is referenced in .github/skills/README.md."""
        issues = []
        
        # Determine if this is in .github/skills or .claude/skills
        path_parts = Path(skill_path).parts
        if '.github' in path_parts and 'skills' in path_parts:
            readme_path = os.path.join(self.repo_root, '.github', 'skills', 'README.md')
        elif '.claude' in path_parts and 'skills' in path_parts:
            readme_path = os.path.join(self.repo_root, '.claude', 'skills', 'README.md')
        else:
            # Not in expected location, skip this check
            return issues
        
        if not os.path.exists(readme_path):
            if self.show_warnings:
                issues.append(self.add_warning(
                    f"Skills README not found at {readme_path}",
                    section="xref"
                ))
            return issues
        
        with open(readme_path, 'r', encoding='utf-8') as f:
            readme_content = f.read()
        
        # Check if skill name appears in README
        if skill_name not in readme_content.lower():
            issues.append(self.add_warning(
                f"Skill '{skill_name}' not found in {readme_path}. Consider adding it to the skills catalog.",
                section="xref"
            ) if self.show_warnings else None)
        
        # Filter out None
        return [i for i in issues if i is not None]
    
    def _check_internal_links(self, skill_path: str, body: str, skill_dir: str) -> List[ValidationIssue]:
        """Check internal markdown links in the skill body."""
        issues = []
        
        # Find all markdown links
        link_pattern = r'\[([^\]]+)\]\(([^)]+)\)'
        matches = re.findall(link_pattern, body)
        
        for link_text, link_url in matches:
            # Skip external links (http/https)
            if link_url.startswith(('http://', 'https://', '#')):
                continue
            
            # Resolve relative links
            if link_url.startswith('./') or link_url.startswith('../'):
                target_path = os.path.normpath(os.path.join(skill_dir, link_url))
            else:
                target_path = os.path.join(skill_dir, link_url)
            
            # Check if target exists
            if not os.path.exists(target_path):
                issues.append(self.add_error(
                    f"Broken internal link: '{link_url}' (target not found: {target_path})",
                    section="xref"
                ))
        
        return issues
