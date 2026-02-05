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

"""Validator for skill description patterns (WHAT/WHEN/KEYWORDS)."""

import re
from typing import List, Set

from . import SkillValidator, ValidationIssue


class DescriptionValidator(SkillValidator):
    """Validates description follows WHAT/WHEN/KEYWORDS pattern."""
    
    # Vague/generic terms that indicate poor descriptions
    VAGUE_TERMS = {
        'helpers', 'utilities', 'tools', 'functions', 'methods',
        'stuff', 'things', 'misc', 'various', 'general'
    }
    
    # Trigger phrases that indicate WHEN guidance
    WHEN_INDICATORS = {
        'use when', 'when asked', 'when you need', 'when working',
        'when debugging', 'when testing', 'when building', 'when creating',
        'for', 'to verify', 'to debug', 'to test', 'to build', 'to analyze',
        'supports', 'includes', 'provides'
    }
    
    # Overly broad keywords that might cause false activations
    BROAD_KEYWORDS = {
        'file', 'code', 'test', 'build', 'run', 'help',
        'make', 'create', 'write', 'read', 'update'
    }
    
    MIN_UNIQUE_KEYWORDS = 5
    
    def validate(self, skill_path: str, content: str, frontmatter: dict, body: str) -> List[ValidationIssue]:
        """Validate description pattern."""
        issues = []
        
        if 'description' not in frontmatter:
            return issues  # Handled by frontmatter validator
        
        description = frontmatter['description']
        if not description:
            return issues  # Handled by frontmatter validator
        
        # Check for vague terms
        desc_lower = description.lower()
        found_vague = [term for term in self.VAGUE_TERMS if term in desc_lower.split()]
        if found_vague:
            issues.append(self.add_error(
                f"Description contains vague terms that hinder discovery: {', '.join(found_vague)}. "
                f"Be specific about capabilities and use cases.",
                section="description"
            ))
        
        # Check for WHEN indicators (triggers/scenarios)
        has_when = any(indicator in desc_lower for indicator in self.WHEN_INDICATORS)
        if not has_when:
            issues.append(self.add_error(
                "Description missing WHEN guidance. Include specific triggers like 'Use when asked to...', "
                "'for debugging...', 'to verify...', etc.",
                section="description"
            ))
        
        # Extract and check keywords
        keywords = self._extract_keywords(description)
        if len(keywords) < self.MIN_UNIQUE_KEYWORDS:
            issues.append(self.add_error(
                f"Description has too few unique keywords ({len(keywords)} found, minimum {self.MIN_UNIQUE_KEYWORDS} required). "
                f"Add specific technical terms, actions, or domain concepts.",
                section="description"
            ))
        
        # Check for overly broad keywords only
        if self.show_warnings:
            broad_only = keywords.intersection(self.BROAD_KEYWORDS)
            if len(broad_only) > len(keywords) * 0.5:  # More than 50% are broad
                issues.append(self.add_warning(
                    f"Description relies heavily on broad keywords that might cause false activations: "
                    f"{', '.join(sorted(broad_only))}. Add more specific technical terms.",
                    section="description"
                ))
        
        # Check for action verbs (capabilities)
        action_verbs = self._count_action_verbs(description)
        if action_verbs < 2:
            issues.append(self.add_warning(
                "Description has few action verbs. Include specific capabilities like 'test', 'debug', "
                "'capture', 'analyze', 'verify', etc.",
                section="description"
            ) if self.show_warnings else None)
        
        # Filter out None values
        issues = [i for i in issues if i is not None]
        
        return issues
    
    def _extract_keywords(self, description: str) -> Set[str]:
        """Extract meaningful keywords from description."""
        # Remove common words and extract technical terms
        words = re.findall(r'\b[a-z][a-z0-9]{2,}\b', description.lower())
        
        # Common stop words to exclude
        stop_words = {
            'the', 'and', 'for', 'with', 'when', 'that', 'this', 'from',
            'use', 'asked', 'need', 'can', 'will', 'are', 'has', 'have'
        }
        
        keywords = {word for word in words if word not in stop_words}
        return keywords
    
    def _count_action_verbs(self, description: str) -> int:
        """Count action verbs in description."""
        action_verbs = {
            'test', 'debug', 'verify', 'validate', 'check', 'analyze', 'capture',
            'build', 'deploy', 'run', 'execute', 'create', 'generate', 'configure',
            'monitor', 'track', 'measure', 'inspect', 'review', 'audit', 'scan'
        }
        
        desc_lower = description.lower()
        return sum(1 for verb in action_verbs if verb in desc_lower)
