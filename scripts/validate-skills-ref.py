#!/usr/bin/env python3
"""Validate repository skills using the upstream skills-ref Python API."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    from skills_ref import validate as validate_skill  # type: ignore
except ImportError:  # pragma: no cover - runtime dependency guard
    validate_skill = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Validate all SKILL.md directories using the skills-ref Python API.'
    )
    parser.add_argument(
        'skill_roots',
        nargs='*',
        default=['.github/skills'],
        help='Root directories that contain skill subdirectories (default: .github/skills).',
    )
    return parser.parse_args()


def find_skill_dirs(root: Path) -> list[Path]:
    if root.is_file() and root.name.lower() == 'skill.md':
        return [root.parent]

    if root.is_dir() and (root / 'SKILL.md').exists():
        return [root]

    if not root.exists():
        return []

    return sorted({path.parent for path in root.glob('**/SKILL.md')})


def main() -> int:
    args = parse_args()

    if validate_skill is None:
        print(
            'ERROR: skills-ref is not installed. Install skills-ref==0.1.1 on Python >=3.11.',
            file=sys.stderr,
        )
        return 2

    skill_dirs: list[Path] = []
    for root_str in args.skill_roots:
        root = Path(root_str)
        dirs = find_skill_dirs(root)
        if not dirs:
            print(f'WARNING: No SKILL.md files found under {root}')
        skill_dirs.extend(dirs)

    unique_skill_dirs = sorted(set(skill_dirs))
    if not unique_skill_dirs:
        print('ERROR: No skill directories found to validate.', file=sys.stderr)
        return 1

    print('Using validator: skills_ref.validate (skills-ref Python API)')
    print(f'Validating {len(unique_skill_dirs)} skill directories...')

    failed = False
    for skill_dir in unique_skill_dirs:
        errors = validate_skill(skill_dir)
        if not errors:
            print(f'PASS: {skill_dir}')
            continue

        failed = True
        print(f'FAIL: {skill_dir}', file=sys.stderr)
        for error in errors:
            print(f'  - {error}', file=sys.stderr)

    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
