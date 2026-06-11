#!/usr/bin/env python3

"""Tests for prompt file validation."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from validate_agent_files.core import CustomizationsValidationEngine
from validate_agent_files.loaders import CustomFileContent
from validate_agent_files.main import main


def test_issue310_prompt_validation_requires_agent_frontmatter_and_non_empty_body(
    tmp_path: Path, capsys
) -> None:
    """Prompt files should require an agent field and non-empty body."""
    (tmp_path / 'missing-agent.prompt.md').write_text(
        """---
model: GPT-5.4
---

# Missing Agent
"""
    )
    (tmp_path / 'empty-body.prompt.md').write_text(
        """---
agent: principal-engineer
model: GPT-5.4
---
"""
    )

    exit_code = main([str(tmp_path), '--kind', 'prompts'])
    captured = capsys.readouterr()

    assert exit_code == 1
    assert 'missing-agent.prompt.md' in captured.out
    assert 'agent' in captured.out
    assert 'empty-body.prompt.md' in captured.out
    assert 'body' in captured.out.lower()


def test_issue310_prompt_validation_checks_file_references_used_by_repo_prompts(
    tmp_path: Path, capsys
) -> None:
    """Prompt validation should understand repo-style #file references."""
    prompt_path = tmp_path / 'broken-reference.prompt.md'
    prompt_path.write_text(
        """---
agent: principal-engineer
model: GPT-5.4
---

# Broken Prompt

Use #file:./missing.instructions.md before implementing.
"""
    )

    exit_code = main([str(tmp_path), '--kind', 'prompts'])
    captured = capsys.readouterr()

    assert exit_code == 1
    assert 'missing.instructions.md' in captured.out


def test_issue310_customizations_engine_reports_prompt_parse_and_no_frontmatter_paths(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Prompt validation should preserve parse failures and plain markdown failures."""
    prompt_files = ['broken.prompt.md', 'plain.prompt.md']

    def fake_load_custom_file(file_path: str) -> CustomFileContent:
        if file_path == 'broken.prompt.md':
            raise yaml.YAMLError('bad yaml')
        return CustomFileContent(
            frontmatter={},
            body='# Plain Prompt\n',
            body_start_line=1,
            has_frontmatter=False,
        )

    monkeypatch.setattr('validate_agent_files.core.find_prompt_files', lambda _: prompt_files)
    monkeypatch.setattr('validate_agent_files.core.load_custom_file', fake_load_custom_file)

    results = CustomizationsValidationEngine()._validate_prompts('unused')

    assert len(results) == 2
    assert results[0].issues[0].section == 'parsing'
    assert results[0].issues[0].message == 'Failed to parse file: bad yaml'
    assert results[1].issues[0].message == 'File must start with YAML frontmatter'
