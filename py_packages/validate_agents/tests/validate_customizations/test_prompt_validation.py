#!/usr/bin/env python3

"""Tests for prompt file validation."""

from __future__ import annotations

from pathlib import Path

from validate_agents.validate_customizations.main import main


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
