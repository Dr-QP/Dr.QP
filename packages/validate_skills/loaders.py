#!/usr/bin/env python3
# Copyright (c) 2026 Dr.QP
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""File loading and parsing utilities for skill validation."""

import os
import logging
import re
from pathlib import Path
from typing import Dict, List, Tuple

import yaml

logger = logging.getLogger(__name__)


def find_skill_files(path: str) -> List[str]:
    """Find all SKILL.md files in a directory or return file if it's a SKILL.md file.

    Args:
        path: Path to directory or file

    Returns:
        List of paths to SKILL.md files
    """
    path = str(path)

    # If it's a file
    if os.path.isfile(path):
        if path.endswith('SKILL.md'):
            return [path]
        return []

    # If path doesn't exist
    if not os.path.exists(path):
        return []

    # It's a directory - find all SKILL.md files
    skill_files = []
    exclude_dirs = {'test', 'tests', '.pytest_cache', '__pycache__', '.git', 'node_modules'}

    for root, dirs, files in os.walk(path):
        # Remove excluded directories from dirs in-place to prevent traversal
        dirs[:] = [d for d in dirs if d not in exclude_dirs]

        if 'SKILL.md' in files:
            skill_files.append(os.path.join(root, 'SKILL.md'))

    return sorted(skill_files)


def safe_load_frontmatter(file_path: str) -> Tuple[Dict, str]:
    """Safely load YAML frontmatter and body from a file.

    Args:
        file_path: Path to the skill file

    Returns:
        Tuple of (frontmatter_dict, body_str)

    Raises:
        OSError: If file not found or cannot be read
        UnicodeDecodeError: If file encoding is invalid
        yaml.YAMLError: If frontmatter delimiters are incomplete
    """
    file_stat = os.stat(file_path)
    if file_stat.st_mode & 0o444 == 0:
        raise PermissionError(f'Permission denied: {file_path}')

    # This will raise OSError if file not found
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Remove BOM if present
    if content.startswith('\ufeff'):
        content = content[1:]

    # Split on frontmatter delimiter
    if not content.startswith('---'):
        # File doesn't have frontmatter, return empty dict and full content as body
        return {}, content

    parts = content.split('---', 2)
    if len(parts) < 3:
        # Incomplete frontmatter - opening delimiter found but no closing
        raise yaml.YAMLError('Incomplete frontmatter delimiters in file')

    frontmatter_str = parts[1]
    body = parts[2].lstrip('\n')

    frontmatter_str = _normalize_frontmatter(frontmatter_str)

    # Parse YAML frontmatter
    try:
        frontmatter = yaml.safe_load(frontmatter_str) or {}
    except yaml.YAMLError as e:
        # If YAML parsing fails, raise the error
        raise yaml.YAMLError(f'Failed to parse frontmatter YAML: {e}')

    return frontmatter, body


def _normalize_frontmatter(frontmatter_str: str) -> str:
    """Normalize common frontmatter formatting issues.

    This keeps YAML strict for general structure while tolerating
    unquoted colons or quotes in name/description values.
    """
    lines = frontmatter_str.splitlines()
    normalized_lines: List[str] = []

    for line in lines:
        match = re.match(r'^(name|description):\s*(.*)$', line)
        if not match:
            normalized_lines.append(line)
            continue

        key = match.group(1)
        value = match.group(2)

        if not value:
            normalized_lines.append(line)
            continue

        stripped = value.strip()
        if key == 'description' and value.count(':') > 1:
            normalized_lines.append(line)
            continue

        if stripped.startswith(('"', "'")):
            if stripped.startswith('"') and stripped.endswith('"'):
                inner = stripped[1:-1]
                escaped = inner.replace('"', '\\"')
                normalized_lines.append(f'{key}: "{escaped}"')
                continue

            normalized_lines.append(line)
            continue

        if ':' in value or '"' in value:
            escaped = value.replace('"', '\\"')
            normalized_lines.append(f'{key}: "{escaped}"')
            continue

        normalized_lines.append(line)

    return '\n'.join(normalized_lines)


def load_all_skills(skill_files: List[str]) -> Dict[str, Dict]:
    """Load multiple skill files.

    Args:
        skill_files: List of paths to skill files

    Returns:
        Dict of {file_path: {frontmatter, body}}
    """
    skills = {}

    for file_path in skill_files:
        try:
            frontmatter, body = safe_load_frontmatter(file_path)
            skills[file_path] = {
                'frontmatter': frontmatter,
                'body': body,
            }
        except Exception as e:
            logger.warning(f'Failed to load skill {file_path}: {e}')

    return skills


class SkillFileLoader:
    """Loads and parses skill files."""

    def __init__(self):
        """Initialize the loader."""
        pass

    def find_skill_files(self, path: str) -> List[str]:
        """Find all SKILL.md files in a directory.

        Args:
            path: Path to search

        Returns:
            List of skill file paths
        """
        return find_skill_files(path)

    def load_skill(self, path: str) -> Tuple[Dict, str]:
        """Load a single skill file.

        Args:
            path: Path to the skill file

        Returns:
            Tuple of (frontmatter, body)
        """
        return safe_load_frontmatter(path)
