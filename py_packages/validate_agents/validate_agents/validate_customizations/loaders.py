#!/usr/bin/env python3

"""File discovery and parsing helpers for customization validation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List

import yaml

from ..validate_skills.loaders import safe_load_frontmatter_with_body_line

EXCLUDED_DIRS = {'test', 'tests', '.pytest_cache', '__pycache__', '.git', 'node_modules'}


@dataclass
class CustomFileContent:
    """Parsed contents for a customization markdown file."""

    frontmatter: dict
    body: str
    body_start_line: int
    has_frontmatter: bool


def _find_matching_files(path: str, suffix: str) -> List[str]:
    candidate = Path(path)
    if candidate.is_file():
        return [str(candidate)] if candidate.name.endswith(suffix) else []
    if not candidate.exists():
        return []

    matches: List[str] = []
    for root, dirs, files in candidate.walk():
        dirs[:] = [dirname for dirname in dirs if dirname not in EXCLUDED_DIRS]
        for filename in files:
            if filename.endswith(suffix):
                matches.append(str(root / filename))

    return sorted(matches)


def find_agent_files(path: str) -> List[str]:
    """Find all .agent.md files under a path."""
    return _find_matching_files(path, '.agent.md')


def find_prompt_files(path: str) -> List[str]:
    """Find all .prompt.md files under a path."""
    return _find_matching_files(path, '.prompt.md')


def load_custom_file(file_path: str) -> CustomFileContent:
    """Load frontmatter and body for an agent or prompt markdown file."""
    raw_content = Path(file_path).read_text(encoding='utf-8')
    normalized = raw_content[1:] if raw_content.startswith('\ufeff') else raw_content
    has_frontmatter = normalized.startswith('---')

    try:
        frontmatter, body, body_start_line = safe_load_frontmatter_with_body_line(file_path)
    except (OSError, UnicodeDecodeError, yaml.YAMLError):
        raise

    return CustomFileContent(
        frontmatter=frontmatter,
        body=body,
        body_start_line=body_start_line,
        has_frontmatter=has_frontmatter,
    )


def agent_identifier(file_path: str) -> str:
    """Return the identifier implied by an agent file name."""
    path = Path(file_path)
    return path.name.removesuffix('.agent.md')
