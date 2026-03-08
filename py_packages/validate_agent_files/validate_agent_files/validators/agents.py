#!/usr/bin/env python3

"""Validation rules for .agent.md files."""

from __future__ import annotations

from typing import Dict, Iterable, List, Set

from ..types import ValidationIssue, ValidationLevel


def validate_agent_frontmatter(
    frontmatter: dict, known_targets: Set[str]
) -> List[ValidationIssue]:
    issues: List[ValidationIssue] = []

    description = frontmatter.get('description')
    if not isinstance(description, str) or not description.strip():
        issues.append(
            ValidationIssue(
                level=ValidationLevel.ERROR,
                message='Missing required field in frontmatter: description',
                section='frontmatter',
            )
        )

    model = frontmatter.get('model')
    if model is not None and not isinstance(model, str):
        issues.append(
            ValidationIssue(
                level=ValidationLevel.ERROR,
                message="Field 'model' must be a string",
                section='frontmatter',
            )
        )

    tools = frontmatter.get('tools')
    if tools is not None and not isinstance(tools, (str, list)):
        issues.append(
            ValidationIssue(
                level=ValidationLevel.ERROR,
                message="Field 'tools' must be a string or list",
                section='frontmatter',
            )
        )

    handoffs = frontmatter.get('handoffs')
    if handoffs is None:
        return issues

    if not isinstance(handoffs, list):
        issues.append(
            ValidationIssue(
                level=ValidationLevel.ERROR,
                message="Field 'handoffs' must be a list",
                section='handoffs',
            )
        )
        return issues

    for index, handoff in enumerate(handoffs, start=1):
        issues.extend(validate_handoff(handoff, known_targets, index))

    return issues


def validate_handoff(
    handoff: object, known_targets: Set[str], index: int
) -> Iterable[ValidationIssue]:
    if not isinstance(handoff, dict):
        yield ValidationIssue(
            level=ValidationLevel.ERROR,
            message=f'Handoff #{index} must be a mapping',
            section='handoffs',
        )
        return

    label = handoff.get('label')
    if not isinstance(label, str) or not label.strip():
        yield ValidationIssue(
            level=ValidationLevel.ERROR,
            message=f"Handoff #{index} is missing required field 'label'",
            section='handoffs',
        )

    agent = handoff.get('agent')
    if not isinstance(agent, str) or not agent.strip():
        yield ValidationIssue(
            level=ValidationLevel.ERROR,
            message=f"Handoff #{index} is missing required field 'agent'",
            section='handoffs',
        )
    elif known_targets and agent not in known_targets:
        yield ValidationIssue(
            level=ValidationLevel.ERROR,
            message=f"Handoff #{index} references unknown agent '{agent}'",
            section='handoffs',
        )

    prompt = handoff.get('prompt')
    if prompt is not None and not isinstance(prompt, str):
        yield ValidationIssue(
            level=ValidationLevel.ERROR,
            message=f"Handoff #{index} field 'prompt' must be a string",
            section='handoffs',
        )

    send = handoff.get('send')
    if send is not None and not isinstance(send, bool):
        yield ValidationIssue(
            level=ValidationLevel.ERROR,
            message=f"Handoff #{index} field 'send' must be a boolean",
            section='handoffs',
        )


def build_known_agent_targets(agent_documents: Dict[str, dict]) -> Set[str]:
    targets: Set[str] = set()
    for file_path, frontmatter in agent_documents.items():
        targets.add(file_path)
        identifier = frontmatter.get('_identifier')
        if isinstance(identifier, str) and identifier.strip():
            targets.add(identifier.strip())
        name = frontmatter.get('name')
        if isinstance(name, str) and name.strip():
            targets.add(name.strip())
    return targets
