#!/usr/bin/env python3

"""Validation engine for repository customizations."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List

import yaml

from ..validate_skills.core import ValidationEngine
from ..validate_skills.loaders import load_all_skills, SkillFileLoader
from ..validate_skills.types import ValidationIssue, ValidationLevel, ValidationResult
from ..validate_skills.validators.cross_reference import CrossReferenceValidator
from .loaders import agent_identifier, find_agent_files, find_prompt_files, load_custom_file
from .validators.agents import build_known_agent_targets, validate_agent_frontmatter
from .validators.prompts import (
    validate_prompt_body,
    validate_prompt_frontmatter,
    validate_prompt_references,
)


class CustomizationsValidationEngine:
    """Orchestrates validation for skills, agents, and prompts."""

    def __init__(self, show_warnings: bool = False):
        self.show_warnings = show_warnings

    def validate(self, path: str, kind: str) -> List[ValidationResult]:
        """Validate customizations found under a path."""
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
                frontmatter['_identifier'] = agent_identifier(file_path)
                agent_documents[file_path] = frontmatter

        known_targets = build_known_agent_targets(agent_documents)
        results: List[ValidationResult] = []
        for file_path in agent_files:
            if file_path in parse_errors:
                results.append(parse_errors[file_path])
                continue

            results.append(self._validate_agent_file(file_path, agent_documents, known_targets))

        return results

    def _validate_agent_file(
        self,
        file_path: str,
        agent_documents: Dict[str, dict],
        known_targets: set[str],
    ) -> ValidationResult:
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
