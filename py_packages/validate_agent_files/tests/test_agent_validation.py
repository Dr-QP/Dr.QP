#!/usr/bin/env python3

"""Tests for repo-specific agent file validation."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from validate_agent_files.core import CustomizationsValidationEngine
from validate_agent_files.loaders import CustomFileContent
from validate_agent_files.main import main
from validate_agent_files.validators.agents import (
    build_known_agent_targets,
    validate_agent_frontmatter,
)
from validate_agent_files.validators.cross_reference import CrossReferenceValidator


def test_issue310_agent_validation_requires_repo_frontmatter_and_handoff_shape(
    tmp_path: Path, capsys
) -> None:
    """Agent files should enforce required repo schema fields."""
    valid_agent = tmp_path / 'valid.agent.md'
    valid_agent.write_text(
        """---
name: Valid Agent
description: Valid description.
model: GPT-5.4
tools: [read]
handoffs:
  - label: Next
    agent: other-agent
---

# Valid Agent
"""
    )

    missing_description = tmp_path / 'missing-description.agent.md'
    missing_description.write_text(
        """---
name: Missing Description
model: GPT-5.4
tools: [read]
---

# Missing Description
"""
    )

    bad_handoff = tmp_path / 'bad-handoff.agent.md'
    bad_handoff.write_text(
        """---
name: Bad Handoff
description: Valid description.
model: GPT-5.4
tools: [read]
handoffs:
  - label: Missing target
---

# Bad Handoff
"""
    )

    exit_code = main([str(tmp_path), '--kind', 'agents'])
    captured = capsys.readouterr()

    assert exit_code == 1
    assert 'missing-description.agent.md' in captured.out
    assert 'description' in captured.out
    assert 'bad-handoff.agent.md' in captured.out
    assert 'handoffs' in captured.out


def test_issue310_agent_validation_checks_handoff_target_exists(tmp_path: Path, capsys) -> None:
    """Handoffs should resolve to an existing agent by file stem or name."""
    (tmp_path / 'existing.agent.md').write_text(
        """---
name: Existing Agent
description: Valid description.
model: GPT-5.4
tools: [read]
---

# Existing Agent
"""
    )
    (tmp_path / 'missing-target.agent.md').write_text(
        """---
name: Missing Target
description: Valid description.
model: GPT-5.4
tools: [read]
handoffs:
  - label: Go missing
    agent: does-not-exist
---

# Missing Target
"""
    )

    exit_code = main([str(tmp_path), '--kind', 'agents'])
    captured = capsys.readouterr()

    assert exit_code == 1
    assert 'does-not-exist' in captured.out


def test_issue310_agent_validation_accepts_stem_handoff_targets(tmp_path: Path, capsys) -> None:
    """Handoffs should accept the target agent file stem as an identifier."""
    (tmp_path / 'principal-engineer.agent.md').write_text(
        """---
name: Principal Engineer
description: Valid description.
model: GPT-5.4
tools: [read]
---

# Principal Engineer
"""
    )
    (tmp_path / 'delegator.agent.md').write_text(
        """---
name: Delegator
description: Valid description.
model: GPT-5.4
tools: [read]
handoffs:
  - label: Delegate
    agent: principal-engineer
---

# Delegator
"""
    )

    exit_code = main([str(tmp_path), '--kind', 'agents'])
    captured = capsys.readouterr()

    assert exit_code == 0
    assert 'references unknown agent' not in captured.out


def test_issue310_customizations_engine_reports_agent_parse_and_no_frontmatter_paths(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Agent validation should preserve parsing failures and missing frontmatter errors."""
    agent_files = ['broken.agent.md', 'plain.agent.md']

    def fake_load_custom_file(file_path: str) -> CustomFileContent:
        if file_path == 'broken.agent.md':
            raise yaml.YAMLError('bad yaml')
        return CustomFileContent(
            frontmatter={},
            body='# Plain Agent\n',
            body_start_line=1,
            has_frontmatter=False,
        )

    monkeypatch.setattr('validate_agent_files.core.find_agent_files', lambda _: agent_files)
    monkeypatch.setattr('validate_agent_files.core.load_custom_file', fake_load_custom_file)

    results = CustomizationsValidationEngine()._validate_agents('unused')

    assert len(results) == 2
    assert results[0].issues[0].section == 'parsing'
    assert 'Failed to parse file: bad yaml' == results[0].issues[0].message
    assert results[1].issues[0].message == 'File must start with YAML frontmatter'


def test_issue310_validate_agent_frontmatter_and_handoffs_reject_invalid_shapes() -> None:
    """Agent frontmatter validation should cover type and handoff shape errors."""
    issues = validate_agent_frontmatter(
        {
            'description': '   ',
            'model': 12,
            'tools': {'read': True},
            'handoffs': 'principal-engineer',
        },
        known_targets=set(),
    )

    assert [issue.message for issue in issues] == [
        'Missing required field in frontmatter: description',
        "Field 'model' must be a string",
        "Field 'tools' must be a string or list",
        "Field 'handoffs' must be a list",
    ]


@pytest.mark.parametrize(
    ('handoff', 'message'),
    [
        ('invalid', 'Handoff #1 must be a mapping'),
        ({'agent': 'principal-engineer'}, "Handoff #1 is missing required field 'label'"),
        ({'label': 'Delegate'}, "Handoff #1 is missing required field 'agent'"),
        (
            {'label': 'Delegate', 'agent': 'missing'},
            "Handoff #1 references unknown agent 'missing'",
        ),
        (
            {'label': 'Delegate', 'agent': 'principal-engineer', 'prompt': 3},
            "Handoff #1 field 'prompt' must be a string",
        ),
        (
            {'label': 'Delegate', 'agent': 'principal-engineer', 'send': 'yes'},
            "Handoff #1 field 'send' must be a boolean",
        ),
    ],
)
def test_issue310_validate_agent_frontmatter_rejects_invalid_handoffs(
    handoff: object,
    message: str,
) -> None:
    """Each handoff validation branch should produce a specific error."""
    issues = validate_agent_frontmatter(
        {
            'description': 'Valid description.',
            'handoffs': [handoff],
        },
        known_targets={'principal-engineer'},
    )

    assert issues[0].message == message


def test_issue310_build_known_agent_targets_collects_file_identifier_and_name() -> None:
    """Known targets should include the path, identifier, and human-readable name."""
    targets = build_known_agent_targets(
        {
            '/tmp/demo.agent.md': {'_identifier': 'demo', 'name': 'Demo Agent'},
            '/tmp/blank.agent.md': {'_identifier': '  ', 'name': '  '},
        }
    )

    assert '/tmp/demo.agent.md' in targets
    assert 'demo' in targets
    assert 'Demo Agent' in targets
    assert '/tmp/blank.agent.md' in targets


def test_issue310_cross_reference_validator_skips_ignored_external_anchor_and_placeholder_links(
    tmp_path: Path,
) -> None:
    """Cross-reference validation should ignore supported non-file references."""
    validator = CrossReferenceValidator(base_path=str(tmp_path))
    content = '\n'.join(
        [
            '<!-- validate_skills: ignore-cross-reference-start -->',
            '[ignored](missing-inside-ignore.md)',
            '<!-- validate_skills: ignore-cross-reference-end -->',
            '[external](https://example.com)',
            '[anchor](#section)',
            '[placeholder](path/to/example)',
            '[broken](missing.md)',
        ]
    )

    issues = validator.validate('skill.md', {}, content, line_offset=5)

    assert len(issues) == 1
    assert issues[0].message == 'Broken reference: missing.md'
    assert issues[0].line_number == 12
    assert issues[0].column_number == 1


def test_issue310_cross_reference_validator_warns_on_resolution_failure_and_helper_edges(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Cross-reference helpers should cover warning and range-building branches."""
    validator = CrossReferenceValidator(base_path=str(tmp_path))
    original_resolve_reference = CrossReferenceValidator._resolve_reference

    class BrokenPath:
        def resolve(self):
            raise ValueError('bad path')

    monkeypatch.setattr(
        CrossReferenceValidator,
        '_resolve_reference',
        staticmethod(lambda base, reference: BrokenPath()),
    )

    issues = validator.validate('skill.md', {}, '[broken](missing.md)')

    assert len(issues) == 1
    assert issues[0].level.value == 'warning'
    assert issues[0].message == 'Broken reference: missing.md'

    resolved = original_resolve_reference(Path(tmp_path), '/docs/readme.md')
    assert resolved == (Path.cwd() / 'docs' / 'readme.md').resolve()

    ignored_content = (
        '<!-- validate_skills: ignore-cross-reference-start -->\n[ignored](missing.md)'
    )
    ignored_ranges = CrossReferenceValidator._build_ignored_ranges(ignored_content)
    assert ignored_ranges == [(0, len(ignored_content))]
    assert CrossReferenceValidator._is_ignored_position(10, ignored_ranges) is True
    assert CrossReferenceValidator._is_ignored_position(100, ignored_ranges) is False
