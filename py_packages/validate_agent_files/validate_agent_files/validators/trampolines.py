#!/usr/bin/env python3

"""
Cross-catalog validation for Codex agent trampolines.

Canonical agents live in ``.claude/agents/<stem>.agent.md``. Each one must have
a matching Codex trampoline at ``.codex/agents/<stem>.md`` whose ``name`` and
``description`` frontmatter fields are identical to the canonical file's. Only
convention keeps these in sync, so this validator makes drift fail validation.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Tuple

import yaml

from ..loaders import safe_load_frontmatter
from ..types import ValidationIssue, ValidationLevel, ValidationResult


def _canonical_agents_dir(agent_file: str) -> Path | None:
    """Return the ``.claude/agents`` directory if ``agent_file`` lives in one."""
    parent = Path(agent_file).parent
    if parent.name == 'agents' and parent.parent.name == '.claude':
        return parent
    return None


def _codex_agents_dir(canonical_agents_dir: Path) -> Path:
    """Return the sibling ``.codex/agents`` directory for a ``.claude/agents`` dir."""
    repo_root = canonical_agents_dir.parent.parent
    return repo_root / '.codex' / 'agents'


def _trampoline_path(agent_file: str) -> Path | None:
    """Return the expected Codex trampoline path for a canonical agent file."""
    canonical_dir = _canonical_agents_dir(agent_file)
    if canonical_dir is None:
        return None
    stem = Path(agent_file).name.removesuffix('.agent.md')
    return _codex_agents_dir(canonical_dir) / f'{stem}.md'


def validate_trampolines(
    agent_files: List[str],
    agent_documents: Dict[str, dict],
) -> Tuple[Dict[str, List[ValidationIssue]], List[ValidationResult]]:
    """
    Validate Codex trampolines against their canonical ``.claude`` agents.

    Returns a mapping of canonical agent path to trampoline issues, plus a list
    of standalone results for orphan trampolines (Codex files without a
    canonical counterpart). Files outside a ``.claude/agents`` directory are
    ignored so ad-hoc agent files (e.g. in tests) need no trampoline.
    """
    per_file_issues: Dict[str, List[ValidationIssue]] = {}
    codex_dir: Path | None = None
    expected_stems: set[str] = set()

    for agent_file in agent_files:
        canonical_dir = _canonical_agents_dir(agent_file)
        if canonical_dir is None:
            continue
        if codex_dir is None:
            codex_dir = _codex_agents_dir(canonical_dir)
            expected_stems.update(
                path.name.removesuffix('.agent.md') for path in canonical_dir.glob('*.agent.md')
            )

        stem = Path(agent_file).name.removesuffix('.agent.md')
        expected_stems.add(stem)
        frontmatter = agent_documents.get(agent_file, {})
        per_file_issues[agent_file] = _validate_single_trampoline(agent_file, frontmatter)

    orphan_results = _find_orphan_trampolines(codex_dir, expected_stems)
    return per_file_issues, orphan_results


def _validate_single_trampoline(
    agent_file: str, canonical_frontmatter: dict
) -> List[ValidationIssue]:
    issues: List[ValidationIssue] = []
    trampoline_path = _trampoline_path(agent_file)
    if trampoline_path is None:
        return issues

    if not trampoline_path.exists():
        issues.append(
            ValidationIssue(
                level=ValidationLevel.ERROR,
                message=(
                    f'Missing Codex trampoline: expected {trampoline_path} for '
                    f'canonical agent {agent_file}'
                ),
                section='trampoline',
            )
        )
        return issues

    try:
        trampoline_frontmatter, _ = safe_load_frontmatter(str(trampoline_path))
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:
        issues.append(
            ValidationIssue(
                level=ValidationLevel.ERROR,
                message=f'Failed to parse Codex trampoline {trampoline_path}: {exc}',
                section='trampoline',
            )
        )
        return issues

    for field in ('name', 'description'):
        canonical_value = canonical_frontmatter.get(field)
        trampoline_value = trampoline_frontmatter.get(field)
        if canonical_value != trampoline_value:
            issues.append(
                ValidationIssue(
                    level=ValidationLevel.ERROR,
                    message=(
                        f"Codex trampoline {trampoline_path} '{field}' does not match "
                        f'canonical agent {agent_file}'
                    ),
                    section='trampoline',
                )
            )

    return issues


def _find_orphan_trampolines(
    codex_dir: Path | None, expected_stems: set[str]
) -> List[ValidationResult]:
    if codex_dir is None or not codex_dir.is_dir():
        return []

    orphan_results: List[ValidationResult] = []
    for trampoline in sorted(codex_dir.glob('*.md')):
        if trampoline.stem in expected_stems:
            continue
        orphan_results.append(
            ValidationResult(
                skill_path=str(trampoline),
                issues=[
                    ValidationIssue(
                        level=ValidationLevel.ERROR,
                        message=(
                            f'Orphan Codex trampoline {trampoline} has no canonical '
                            f'.claude/agents/{trampoline.stem}.agent.md'
                        ),
                        section='trampoline',
                    )
                ],
            )
        )
    return orphan_results
