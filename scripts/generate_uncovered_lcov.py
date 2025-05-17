#!/usr/bin/env python3
import os

# Directory to scan
TARGET_DIR = os.path.join(os.path.dirname(__file__), '../packages/runtime')
# Output file
OUTPUT_FILE = os.path.join(os.path.dirname(__file__), '../lcov.info')

# File extensions to include
CPP_EXTS = {'.cpp', '.cc', '.cxx', '.c', '.h', '.hpp', '.hxx'}
PY_EXTS = {'.py'}


def is_source_file(filename):
    _, ext = os.path.splitext(filename)
    return ext in CPP_EXTS or ext in PY_EXTS


def lcov_section_for_file(filepath):
    # Get absolute path
    abs_path = os.path.abspath(filepath)
    # Read lines
    try:
        with open(abs_path, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
    except (FileNotFoundError, IOError):
        return None
    section = []
    section.append(f'SF:{abs_path}')
    _, ext = os.path.splitext(filepath)
    in_block_comment = False
    for idx, line in enumerate(lines):
        stripped = line.strip()
        # Skip empty lines
        if not stripped:
            continue
        # Python comment
        if ext in PY_EXTS:
            if stripped.startswith('#'):
                continue
        # C++ comments
        elif ext in CPP_EXTS:
            # Handle block comments
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
    lcov_sections = []
    for root, _, files in os.walk(TARGET_DIR):
        for file in files:
            if is_source_file(file):
                full_path = os.path.join(root, file)
                section = lcov_section_for_file(full_path)
                if section:
                    lcov_sections.append(section)
    with open(OUTPUT_FILE, 'w', encoding='utf-8') as out:
        out.write('\n'.join(lcov_sections))
    print(f'LCOV info written to {OUTPUT_FILE}')


if __name__ == '__main__':
    main()
