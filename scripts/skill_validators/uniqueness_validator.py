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

"""Validator for skill uniqueness (duplicate names, overlapping keywords)."""

import re
from typing import Dict, List, Set

from . import SkillValidator, ValidationIssue


class UniquenessValidator(SkillValidator):
    """Validates skills are unique (no duplicate names or excessive keyword overlap)."""
    
    KEYWORD_OVERLAP_THRESHOLD = 0.7  # 70% overlap is considered too much
    
    def __init__(self, show_warnings: bool = False, all_skills: Dict[str, dict] = None):
        """Initialize uniqueness validator.
        
        Args:
            show_warnings: Whether to include warnings
            all_skills: Dict of all skills {path: {frontmatter, body}} for cross-skill validation
        """
        super().__init__(show_warnings)
        self.all_skills = all_skills or {}
    
    def validate(self, skill_path: str, content: str, frontmatter: dict, body: str) -> List[ValidationIssue]:
        """Validate skill uniqueness."""
        issues = []
        
        if not frontmatter or 'name' not in frontmatter:
            return issues  # Handled by frontmatter validator
        
        current_name = frontmatter.get('name')
        current_desc = frontmatter.get('description', '')
        
        # Check for duplicate names
        for other_path, other_data in self.all_skills.items():
            if other_path == skill_path:
                continue
            
            other_fm = other_data.get('frontmatter', {})
            other_name = other_fm.get('name')
            
            if other_name == current_name:
                issues.append(self.add_error(
                    f"Duplicate skill name '{current_name}' found in {other_path}",
                    section="uniqueness"
                ))
        
        # Check for overlapping descriptions/keywords
        if self.show_warnings and current_desc:
            current_keywords = self._extract_keywords(current_desc)
            
            for other_path, other_data in self.all_skills.items():
                if other_path == skill_path:
                    continue
                
                other_fm = other_data.get('frontmatter', {})
                other_desc = other_fm.get('description', '')
                other_keywords = self._extract_keywords(other_desc)
                
                if current_keywords and other_keywords:
                    overlap = self._calculate_overlap(current_keywords, other_keywords)
                    if overlap >= self.KEYWORD_OVERLAP_THRESHOLD:
                        issues.append(self.add_warning(
                            f"High keyword overlap ({overlap:.0%}) with skill in {other_path}. "
                            f"This might cause activation conflicts.",
                            section="uniqueness"
                        ))
        
        return issues
    
    def _extract_keywords(self, description: str) -> Set[str]:
        """Extract keywords from description."""
        words = re.findall(r'\b[a-z][a-z0-9]{2,}\b', description.lower())
        stop_words = {
            'the', 'and', 'for', 'with', 'when', 'that', 'this', 'from',
            'use', 'asked', 'need', 'can', 'will', 'are', 'has', 'have'
        }
        return {word for word in words if word not in stop_words}
    
    def _calculate_overlap(self, keywords1: Set[str], keywords2: Set[str]) -> float:
        """Calculate Jaccard similarity between two keyword sets."""
        if not keywords1 or not keywords2:
            return 0.0
        
        intersection = len(keywords1.intersection(keywords2))
        union = len(keywords1.union(keywords2))
        
        return intersection / union if union > 0 else 0.0
