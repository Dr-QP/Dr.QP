#!/usr/bin/env python3

"""Tests for customization validation entry points."""

from __future__ import annotations

from pathlib import Path
import runpy
import sys
from types import ModuleType

import pytest
import yaml

from validate_agent_files.core import ValidationEngine, skills_ref_validate
try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - Python < 3.11
    import tomli as tomllib

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
    pyproject_path = Path(__file__).resolve().parents[1] / 'pyproject.toml'
    with pyproject_path.open('rb') as stream:
        pyproject = tomllib.load(stream)

    scripts = pyproject['project']['scripts']

    assert scripts == {'validate_agent_files': 'validate_agent_files.__main__:main'}


def test_issue310_validate_agent_files_can_limit_to_skills(
    tmp_path: Path, capsys, monkeypatch
) -> None:
    """The canonical CLI should ignore agents and prompts when kind=skills."""
    monkeypatch.setattr('validate_agent_files.core.skills_ref_validate', lambda _: [])
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
    tmp_path: Path, capsys, monkeypatch
) -> None:
    """The canonical CLI should validate all supported customization kinds."""
    monkeypatch.setattr('validate_agent_files.core.skills_ref_validate', lambda _: [])
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


def test_issue310_validate_agent_files_module_exits_with_main_return_code(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Running the module should exit with the return code from main."""
    monkeypatch.setattr('validate_agent_files.main.main', lambda: 7)
    sys.modules.pop('validate_agent_files.__main__', None)

    with pytest.raises(SystemExit) as excinfo:
        runpy.run_module('validate_agent_files.__main__', run_name='__main__')

    assert excinfo.value.code == 7


def test_issue310_skills_ref_validate_wraps_upstream_validate(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The wrapper should coerce the input to Path and materialize iterable output."""
    seen_paths: list[Path] = []
    fake_module = ModuleType('skills_ref')

    def fake_validate(skill_dir: Path):
        seen_paths.append(skill_dir)
        yield 'first'
        yield 'second'

    fake_module.validate = fake_validate
    monkeypatch.setitem(sys.modules, 'skills_ref', fake_module)

    assert skills_ref_validate('demo-skill') == ['first', 'second']
    assert seen_paths == [Path('demo-skill')]


def test_issue310_validation_engine_returns_parsing_issue_for_invalid_skill(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Skill validation should convert parser failures into a validation result."""
    monkeypatch.setattr(
        'validate_agent_files.core.safe_load_frontmatter_with_body_line',
        lambda _: (_ for _ in ()).throw(yaml.YAMLError('broken yaml')),
    )

    result = ValidationEngine().validate('broken/SKILL.md')

    assert result.is_valid is False
    assert len(result.issues) == 1
    assert result.issues[0].section == 'parsing'
    assert 'Failed to parse file: broken yaml' == result.issues[0].message
