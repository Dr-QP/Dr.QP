#!/usr/bin/env python3

"""Validation engines for skills, agents, and prompts."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional

import yaml

from .loaders import (
    find_agent_files,
    find_prompt_files,
    load_all_skills,
    load_custom_file,
    safe_load_frontmatter_with_body_line,
    SkillFileLoader,
)
from .types import ValidationIssue, ValidationLevel, ValidationResult
from .validators.agents import build_known_agent_targets, validate_agent_frontmatter
from .validators.cross_reference import CrossReferenceValidator
from .validators.prompts import (
    validate_prompt_body,
    validate_prompt_frontmatter,
    validate_prompt_references,
)
from .validators.uniqueness import UniquenessValidator


def skills_ref_validate(skill_dir: Path | str) -> list[str]:
    """Validate a skill directory using the upstream skills-ref package."""
    from skills_ref import validate as validate_skill  # type: ignore[import-not-found]

    skill_dir = Path(skill_dir)
    return list(validate_skill(skill_dir))


class ValidationEngine:
    """Orchestrates validation of skills."""

    def __init__(self, show_warnings: bool = False, show_info: bool = False):
        self.show_warnings = show_warnings
        self.show_info = show_info

    def validate(self, skill_path: str, all_skills: Optional[Dict] = None) -> ValidationResult:
        all_skills = all_skills or {}
        result = ValidationResult(skill_path=skill_path, issues=[])

        try:
            frontmatter, body, body_start_line = safe_load_frontmatter_with_body_line(skill_path)
        except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:
            result.issues.append(
                ValidationIssue(
                    level=ValidationLevel.ERROR,
                    message=f'Failed to parse file: {exc}',
                    section='parsing',
                )
            )
            return result

        skill_dir = Path(skill_path).parent
        for message in skills_ref_validate(skill_dir):
            result.issues.append(
                ValidationIssue(
                    level=ValidationLevel.ERROR,
                    message=message,
                    section='skills_ref',
                )
            )

        if result.issues:
            return result

        unique_validator = UniquenessValidator(all_skills=all_skills)
        result.issues.extend(
            unique_validator.validate(skill_path=skill_path, metadata=frontmatter, content=body)
        )

        xref_validator = CrossReferenceValidator(
            base_path=str(skill_dir), show_warnings=self.show_warnings
        )
        result.issues.extend(
            xref_validator.validate(
                skill_path=skill_path,
                metadata=frontmatter,
                content=body,
                line_offset=body_start_line - 1,
            )
        )

        return result


class CustomizationsValidationEngine:
    """Orchestrates validation for skills, agents, and prompts."""

    def __init__(self, show_warnings: bool = False):
        self.show_warnings = show_warnings

    def validate(self, path: str, kind: str) -> List[ValidationResult]:
        results: List[ValidationResult] = []

        if kind in {'all', 'skills'}:
            results.extend(self._validate_skills(path))
        if kind in {'all', 'agents'}:
            results.extend(self._validate_agents(path))
        if kind in {'all', 'prompts'}:
            results.extend(self._validate_prompts(path))

        return results

    def _validate_skills(self, path: str) -> List[ValidationResult]:
        loader = SkillFileLoader()
        skill_files = loader.find_skill_files(path)
        all_skills = load_all_skills(skill_files)
        engine = ValidationEngine(show_warnings=self.show_warnings)
        return [engine.validate(skill_path, all_skills=all_skills) for skill_path in skill_files]

    def _validate_agents(self, path: str) -> List[ValidationResult]:
        agent_files = find_agent_files(path)
        agent_documents: Dict[str, dict] = {}
        parse_errors: Dict[str, ValidationResult] = {}

        for file_path in agent_files:
            try:
                document = load_custom_file(file_path)
            except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:
                parse_errors[file_path] = ValidationResult(
                    skill_path=file_path,
                    issues=[
                        ValidationIssue(
                            level=ValidationLevel.ERROR,
                            message=f'Failed to parse file: {exc}',
                            section='parsing',
                        )
                    ],
                )
                continue

            if document.has_frontmatter:
                frontmatter = document.frontmatter
                frontmatter['_identifier'] = Path(file_path).name.removesuffix('.agent.md')
                agent_documents[file_path] = frontmatter

        known_targets = build_known_agent_targets(agent_documents)
        results: List[ValidationResult] = []
        for file_path in agent_files:
            if file_path in parse_errors:
                results.append(parse_errors[file_path])
                continue

            results.append(self._validate_agent_file(file_path, known_targets))

        return results

    def _validate_agent_file(self, file_path: str, known_targets: set[str]) -> ValidationResult:
        result = ValidationResult(skill_path=file_path, issues=[])
        document = load_custom_file(file_path)

        if not document.has_frontmatter:
            result.issues.append(
                ValidationIssue(
                    level=ValidationLevel.ERROR,
                    message='File must start with YAML frontmatter',
                    section='parsing',
                )
            )
            return result

        result.issues.extend(validate_agent_frontmatter(document.frontmatter, known_targets))
        xref_validator = CrossReferenceValidator(base_path=str(Path(file_path).parent))
        result.issues.extend(
            xref_validator.validate(
                skill_path=file_path,
                metadata=document.frontmatter,
                content=document.body,
                line_offset=document.body_start_line - 1,
            )
        )
        return result

    def _validate_prompts(self, path: str) -> List[ValidationResult]:
        prompt_files = find_prompt_files(path)
        return [self._validate_prompt_file(file_path) for file_path in prompt_files]

    def _validate_prompt_file(self, file_path: str) -> ValidationResult:
        result = ValidationResult(skill_path=file_path, issues=[])

        try:
            document = load_custom_file(file_path)
        except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:
            result.issues.append(
                ValidationIssue(
                    level=ValidationLevel.ERROR,
                    message=f'Failed to parse file: {exc}',
                    section='parsing',
                )
            )
            return result

        if not document.has_frontmatter:
            result.issues.append(
                ValidationIssue(
                    level=ValidationLevel.ERROR,
                    message='File must start with YAML frontmatter',
                    section='parsing',
                )
            )
            return result

        result.issues.extend(validate_prompt_frontmatter(document.frontmatter))
        result.issues.extend(validate_prompt_body(document.body))

        xref_validator = CrossReferenceValidator(base_path=str(Path(file_path).parent))
        result.issues.extend(
            xref_validator.validate(
                skill_path=file_path,
                metadata=document.frontmatter,
                content=document.body,
                line_offset=document.body_start_line - 1,
            )
        )
        result.issues.extend(
            validate_prompt_references(
                file_path=file_path,
                body=document.body,
                line_offset=document.body_start_line - 1,
            )
        )
        return result


__all__ = [
    'CustomizationsValidationEngine',
    'ValidationEngine',
    'ValidationIssue',
    'ValidationLevel',
    'ValidationResult',
    'skills_ref_validate',
]
