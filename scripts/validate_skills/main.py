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

"""Main orchestration function for the validate-skills tool."""

import logging
import sys
from typing import List, Optional

import yaml

from validate_skills.cli import parse_arguments
from validate_skills.core import ValidationEngine
from validate_skills.formatters import format_results
from validate_skills.loaders import safe_load_frontmatter, SkillFileLoader


def main(args: Optional[List[str]] = None) -> int:
    """
    Run the validate-skills tool.

    Args
    ----
        args: Command-line arguments (if None, sys.argv is used)

    Returns
    -------
        Exit code (0 for success, 1 for validation failures)

    """
    # Parse arguments
    parsed_args = parse_arguments(args)

    # Determine which format to use
    output_format = parsed_args.format

    # Find skill files
    loader = SkillFileLoader()
    skill_files = loader.find_skill_files(parsed_args.path)

    if not skill_files:
        if output_format == 'text':
            print(f'No SKILL.md files found in {parsed_args.path}')
        return 0

    if output_format == 'text':
        print(f'Validating {len(skill_files)} skill(s)...\n')

    # Load all skills for cross-validation
    all_skills = {}
    for skill_path in skill_files:
        try:
            frontmatter, body = safe_load_frontmatter(skill_path)
            with open(skill_path, 'r', encoding='utf-8') as f:
                content = f.read()
            all_skills[skill_path] = {
                'frontmatter': frontmatter,
                'body': body,
                'content': content,
            }
        except (OSError, UnicodeDecodeError, yaml.YAMLError) as e:
            # Log but skip skills that can't be loaded for cross-validation
            logging.warning(f'Failed to load {skill_path}: {e}')

    # Validate each skill
    engine = ValidationEngine(show_warnings=parsed_args.recommend, show_info=parsed_args.recommend)

    results = []
    for skill_path in skill_files:
        result = engine.validate(skill_path, all_skills=all_skills)
        results.append(result)

    # Format and output results
    formatted = format_results(results, output_format=output_format)
    if formatted:
        print(formatted)

    # Return exit code based on results
    failed_count = sum(1 for r in results if not r.is_valid)
    return 1 if failed_count > 0 else 0


if __name__ == '__main__':
    sys.exit(main())
