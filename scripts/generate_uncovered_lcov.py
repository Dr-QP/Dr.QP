#!/usr/bin/env python3
from pathlib import Path
import argparse


def parse_args():
    parser = argparse.ArgumentParser(
        description='Generate LCOV info file marking all code lines as uncovered.'
    )
    parser.add_argument(
        '--repo-root',
        type=Path,
        default=Path(__file__).parent.parent,
        help='Repository root directory',
    )
    parser.add_argument(
        '--target-dir',
        type=Path,
        default=Path('packages') / 'runtime',
        help='Directory to scan for source files',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=Path('build') / 'coverage' / 'all-uncovered' / 'lcov.info',
        help='Output LCOV file',
    )
    args = parser.parse_args()
    # Ensure paths are absolute
    args.repo_root = args.repo_root.resolve()
    if not args.target_dir.is_absolute():
        args.target_dir = (args.repo_root / args.target_dir).resolve()

    if not args.output.is_absolute():
        args.output = (args.repo_root / args.output).resolve()

    # Ensure output dir exists
    if not args.output.parent.exists():
        args.output.parent.mkdir(parents=True, exist_ok=True)

    return args


# File extensions to include
CPP_EXTS = {'.cpp', '.cc', '.cxx', '.c', '.h', '.hpp', '.hxx'}
PY_EXTS = {'.py'}


def is_source_file(filename):
    ext = Path(filename).suffix
    return ext in CPP_EXTS or ext in PY_EXTS


def should_exclude_file(filepath):
    # Exclude files with 'test' in the name or path, or named setup.py
    lower_path = str(filepath).lower()
    filename = Path(filepath).name
    if 'test' in lower_path:
        return True
    if filename == 'setup.py':
        return True
    return False


def lcov_section_for_file(filepath):
    abs_path = Path(filepath).resolve()
    try:
        lines = abs_path.read_text(encoding='utf-8', errors='ignore').splitlines()
    except (FileNotFoundError, IOError):
        return None
    section = []
    section.append(f'SF:{abs_path}')
    ext = abs_path.suffix
    in_block_comment = False
    for idx, line in enumerate(lines):
        stripped = line.strip()
        if not stripped:
            continue
        if ext in PY_EXTS:
            if stripped.startswith('#'):
                continue
        elif ext in CPP_EXTS:
            if in_block_comment:
                if '*/' in stripped:
                    in_block_comment = False
                continue
            if stripped.startswith('//'):
                continue
            if stripped.startswith('/*'):
                in_block_comment = '*/' not in stripped
                continue
            if stripped.startswith('*') and (idx == 0 or lines[idx - 1].strip().startswith('/*')):
                continue
        section.append(f'DA:{idx + 1},0')
    section.append('end_of_record')
    return '\n'.join(section)


def main():
    args = parse_args()
    lcov_sections = []
    target_dir = Path(args.target_dir)
    for file in target_dir.rglob('*'):
        if not file.is_file():
            continue
        if not is_source_file(file):
            continue
        if should_exclude_file(file):
            continue
        section = lcov_section_for_file(file)
        if section:
            lcov_sections.append(section)
    with open(args.output, 'w', encoding='utf-8') as out:
        out.write('\n'.join(lcov_sections))
    print(f'LCOV info written to {args.output}')


if __name__ == '__main__':
    main()
