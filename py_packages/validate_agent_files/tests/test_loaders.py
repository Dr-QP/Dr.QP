#!/usr/bin/env python3

"""Tests for file discovery and frontmatter loading helpers."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from validate_agent_files.loaders import (
    _normalize_frontmatter,
    _parse_frontmatter_content,
    agent_identifier,
    find_agent_files,
    find_prompt_files,
    find_skill_files,
    load_all_skills,
    load_custom_file,
    safe_load_frontmatter,
    safe_load_frontmatter_with_body_line,
    SkillFileLoader,
)


def test_issue310_loader_discovery_and_identifier_helpers_cover_file_cases(
    tmp_path: Path,
) -> None:
    """Discovery helpers should handle valid files, wrong suffixes, and missing paths."""
    skill_file = tmp_path / 'skill' / 'SKILL.md'
    skill_file.parent.mkdir()
    skill_file.write_text('---\nname: demo-skill\ndescription: A valid description.\n---\n# Demo\n')

    agent_file = tmp_path / 'demo.agent.md'
    agent_file.write_text('---\nname: Demo\ndescription: Valid description.\n---\n# Demo\n')

    prompt_file = tmp_path / 'demo.prompt.md'
    prompt_file.write_text('---\nagent: demo\n---\n# Prompt\n')

    plain_file = tmp_path / 'notes.md'
    plain_file.write_text('# Notes\n')

    assert find_skill_files(str(skill_file)) == [str(skill_file)]
    assert find_skill_files(str(plain_file)) == []
    assert find_skill_files(str(tmp_path / 'missing')) == []

    assert find_agent_files(str(agent_file)) == [str(agent_file)]
    assert find_agent_files(str(plain_file)) == []
    assert find_agent_files(str(tmp_path / 'missing.agent.md')) == []

    assert find_prompt_files(str(prompt_file)) == [str(prompt_file)]
    assert find_prompt_files(str(plain_file)) == []
    assert find_prompt_files(str(tmp_path / 'missing.prompt.md')) == []

    loader = SkillFileLoader()
    frontmatter, body = loader.load_skill(str(skill_file))
    assert frontmatter['name'] == 'demo-skill'
    assert body == '# Demo\n'
    assert agent_identifier(str(agent_file)) == 'demo'


@pytest.mark.parametrize(
    ('loader', 'file_name'),
    [
        (safe_load_frontmatter, 'skill.md'),
        (safe_load_frontmatter_with_body_line, 'skill.md'),
    ],
)
def test_issue310_frontmatter_loaders_reject_unreadable_files(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    loader,
    file_name: str,
) -> None:
    """Frontmatter helpers should fail early for unreadable files."""
    file_path = tmp_path / file_name
    file_path.write_text('---\nname: demo\n---\n# Demo\n')
    monkeypatch.setattr(
        'validate_agent_files.loaders.os.stat',
        lambda _: SimpleNamespace(st_mode=0),
    )

    with pytest.raises(PermissionError, match='Permission denied'):
        loader(str(file_path))


def test_issue310_parse_frontmatter_content_handles_bom_and_missing_frontmatter() -> None:
    """The low-level parser should cover BOM and plain body inputs."""
    frontmatter, body, body_start_line = _parse_frontmatter_content(
        '\ufeff---\nname: demo\ndescription: Valid description.\n---\n\n# Demo\n'
    )

    assert frontmatter == {'name': 'demo', 'description': 'Valid description.'}
    assert body == '# Demo\n'
    assert body_start_line == 6

    plain_frontmatter, plain_body, plain_body_start_line = _parse_frontmatter_content('# Body\n')

    assert plain_frontmatter == {}
    assert plain_body == '# Body\n'
    assert plain_body_start_line == 1


def test_issue310_parse_frontmatter_content_rejects_incomplete_and_invalid_yaml() -> None:
    """The parser should raise wrapped YAML errors for malformed frontmatter."""
    with pytest.raises(yaml.YAMLError, match='Incomplete frontmatter delimiters'):
        _parse_frontmatter_content('---\nname: demo\n')

    with pytest.raises(yaml.YAMLError, match='Failed to parse frontmatter YAML'):
        _parse_frontmatter_content('---\nname: [broken\n---\n# Demo\n')


def test_issue310_normalize_frontmatter_handles_repo_edge_cases() -> None:
    """Normalization should preserve supported forms and quote ambiguous values."""
    normalized = _normalize_frontmatter(
        '\n'.join(
            [
                'name: alpha:beta',
                'description: useful: helpers: tools',
                'name: "quoted"',
                "description: 'single quoted'",
                'description: ',
                'name: contains "quotes"',
            ]
        )
    )

    lines = normalized.splitlines()
    assert lines[0] == 'name: "alpha:beta"'
    assert lines[1] == 'description: useful: helpers: tools'
    assert lines[2] == 'name: "quoted"'
    assert lines[3] == "description: 'single quoted'"
    assert lines[4] == 'description: '
    assert lines[5] == 'name: "contains \\"quotes\\""'


def test_issue310_load_all_skills_skips_unparseable_files(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Bulk loading should keep valid skills and log failures for malformed ones."""
    valid_skill = tmp_path / 'valid' / 'SKILL.md'
    valid_skill.parent.mkdir()
    valid_skill.write_text(
        '---\nname: valid-skill\ndescription: A valid description.\n---\n# Valid\n'
    )

    invalid_skill = tmp_path / 'invalid' / 'SKILL.md'
    invalid_skill.parent.mkdir()
    invalid_skill.write_text('---\nname: [broken\n---\n# Invalid\n')

    warnings: list[str] = []
    monkeypatch.setattr('validate_agent_files.loaders.logger.warning', warnings.append)

    loaded = load_all_skills([str(valid_skill), str(invalid_skill)])

    assert str(valid_skill) in loaded
    assert str(invalid_skill) not in loaded
    assert len(warnings) == 1
    assert warnings[0].startswith('Failed to load skill')


def test_issue310_load_custom_file_tracks_frontmatter_presence(tmp_path: Path) -> None:
    """Custom file loading should preserve frontmatter state and body offsets."""
    prompt_with_frontmatter = tmp_path / 'with-frontmatter.prompt.md'
    prompt_with_frontmatter.write_text(
        '---\nagent: demo\nmodel: GPT-5.4\n---\n\n# Prompt\n'
    )
    prompt_without_frontmatter = tmp_path / 'without-frontmatter.prompt.md'
    prompt_without_frontmatter.write_text('# Prompt\n')

    loaded_with_frontmatter = load_custom_file(str(prompt_with_frontmatter))
    loaded_without_frontmatter = load_custom_file(str(prompt_without_frontmatter))

    assert loaded_with_frontmatter.has_frontmatter is True
    assert loaded_with_frontmatter.body_start_line == 6
    assert loaded_without_frontmatter.has_frontmatter is False
    assert loaded_without_frontmatter.frontmatter == {}
    assert loaded_without_frontmatter.body == '# Prompt\n'