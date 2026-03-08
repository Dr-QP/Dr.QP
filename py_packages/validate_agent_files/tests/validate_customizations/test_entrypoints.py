#!/usr/bin/env python3

"""Tests for customization validation entry points."""

from __future__ import annotations

from pathlib import Path
import tomllib

from validate_agent_files.main import main as validate_agent_files_main


def _write_valid_skill(skill_dir: Path, name: str) -> None:
    skill_dir.mkdir(parents=True, exist_ok=True)
    (skill_dir / 'SKILL.md').write_text(
        f"""---
name: {name}
description: A comprehensive description of what this skill does and when to use it.
---
# {name}

See [guide](guide.md).
"""
    )
    (skill_dir / 'guide.md').write_text('# guide\n')


def test_issue310_registers_validate_agent_files_cli() -> None:
    """The package should expose only the canonical validation entry point."""
    pyproject_path = Path(__file__).resolve().parents[2] / 'pyproject.toml'
    with pyproject_path.open('rb') as stream:
        pyproject = tomllib.load(stream)

    scripts = pyproject['project']['scripts']

    assert scripts == {'validate_agent_files': 'validate_agent_files.__main__:main'}


def test_issue310_validate_agent_files_can_limit_to_skills(tmp_path: Path, capsys) -> None:
    """The canonical CLI should ignore agents and prompts when kind=skills."""
    _write_valid_skill(tmp_path / 'valid-skill', 'valid-skill')

    (tmp_path / 'broken.agent.md').write_text(
        """---
name: Broken Agent
tools: [read]
---

# Broken Agent
"""
    )
    (tmp_path / 'broken.prompt.md').write_text(
        """---
model: GPT-5.4
---
"""
    )

    exit_code = validate_agent_files_main([str(tmp_path), '--kind', 'skills'])
    captured = capsys.readouterr()

    assert exit_code == 0
    assert 'broken.agent.md' not in captured.out
    assert 'broken.prompt.md' not in captured.out


def test_issue310_validate_agent_files_scans_skills_agents_and_prompts(
    tmp_path: Path, capsys
) -> None:
    """The canonical CLI should validate all supported customization kinds."""
    _write_valid_skill(tmp_path / 'valid-skill', 'valid-skill')

    (tmp_path / 'broken.agent.md').write_text(
        """---
name: Broken Agent
tools: [read]
---

# Broken Agent
"""
    )
    (tmp_path / 'broken.prompt.md').write_text(
        """---
model: GPT-5.4
---
"""
    )

    exit_code = validate_agent_files_main([str(tmp_path)])
    captured = capsys.readouterr()

    assert exit_code == 1
    assert 'broken.agent.md' in captured.out
    assert 'broken.prompt.md' in captured.out
