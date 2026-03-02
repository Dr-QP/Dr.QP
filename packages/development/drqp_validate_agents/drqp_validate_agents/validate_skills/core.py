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

"""Core validation engine for skills."""

from pathlib import Path
from typing import Dict

import yaml

from .loaders import safe_load_frontmatter
from .types import ValidationIssue, ValidationLevel, ValidationResult
from .validators.cross_reference import CrossReferenceValidator
from .validators.skill import (
    SkillFrontmatterValidator,
    SkillStructureValidator,
)
from .validators.uniqueness import UniquenessValidator


class ValidationEngine:
    """Orchestrates validation of skills."""

    def __init__(self, show_warnings: bool = False, show_info: bool = False):
        """Initialize the validation engine."""
        self.show_warnings = show_warnings
        self.show_info = show_info

    def validate(self, skill_path: str, all_skills: Dict = None) -> ValidationResult:
        """
        Validate a single skill file.

        Args
        ----
            skill_path: Path to the skill file
            all_skills: Dict of all skills for cross-validation

        Returns
        -------
            ValidationResult with any issues found

        """
        all_skills = all_skills or {}
        result = ValidationResult(skill_path=skill_path, issues=[])

        try:
            # Load the file
            frontmatter, body = safe_load_frontmatter(skill_path)
        except (OSError, UnicodeDecodeError, yaml.YAMLError) as e:
            result.issues.append(
                ValidationIssue(
                    level=ValidationLevel.ERROR,
                    message=f'Failed to parse file: {e}',
                    section='parsing',
                )
            )
            return result

        # Validate frontmatter
        fm_validator = SkillFrontmatterValidator()
        result.issues.extend(fm_validator.validate(frontmatter, show_warnings=self.show_warnings))

        # Validate structure
        struct_validator = SkillStructureValidator()
        result.issues.extend(struct_validator.validate(body, show_warnings=self.show_warnings))

        # Validate uniqueness
        unique_validator = UniquenessValidator(all_skills=all_skills)
        result.issues.extend(
            unique_validator.validate(
                skill_path=skill_path,
                metadata=frontmatter,
                content=body,
            )
        )

        # Validate cross-references
        skill_dir = str(Path(skill_path).parent) if skill_path else None
        xref_validator = CrossReferenceValidator(
            base_path=skill_dir, show_warnings=self.show_warnings
        )
        result.issues.extend(
            xref_validator.validate(skill_path=skill_path, metadata=frontmatter, content=body)
        )

        return result
