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

"""Main script for validating Agent Skills."""

import argparse
import os
import sys
from typing import Dict, List

import frontmatter
from pydantic import ValidationError

# Add skill-validators to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

import skill_validators
from skill_validators.models import SkillFrontmatter, SkillStructure
from skill_validators.uniqueness_validator import UniquenessValidator
from skill_validators.xref_validator import CrossReferenceValidator


def find_skill_files(path: str) -> List[str]:
    """Find all SKILL.md files in the given path."""
    if os.path.isfile(path):
        return [path] if path.endswith('SKILL.md') else []

    skill_files = []
    for root, dirs, files in os.walk(path):
        for file in files:
            if file == 'SKILL.md':
                skill_files.append(os.path.join(root, file))

    return skill_files


def load_all_skills(skill_paths: List[str]) -> Dict[str, dict]:
    """
    Load all skills for cross-validation.

    Returns:
        Dict of {path: {frontmatter, body}}

    """
    all_skills = {}

    for path in skill_paths:
        try:
            post = frontmatter.load(path)
            all_skills[path] = {'frontmatter': post.metadata, 'body': post.content}
        except (OSError, UnicodeDecodeError, KeyError, AttributeError) as e:
            print(f'  ⚠ Error loading {path}: {e}')

    return all_skills


def pydantic_errors_to_issues(
    errors: List[dict], section: str
) -> List[skill_validators.ValidationIssue]:
    """Convert Pydantic validation errors to ValidationIssue objects."""
    issues = []
    for error in errors:
        field = '.'.join(str(loc) for loc in error['loc']) if error['loc'] else ''
        message = error['msg']
        if field:
            message = f'{field}: {message}'

        issues.append(
            skill_validators.ValidationIssue(
                level=skill_validators.ValidationLevel.ERROR, message=message, section=section
            )
        )
    return issues


def validate_skill(
    skill_path: str, show_warnings: bool, all_skills: Dict[str, dict], repo_root: str
) -> skill_validators.ValidationResult:
    """Validate a single skill file."""
    issues = []

    try:
        # Parse frontmatter using python-frontmatter
        post = frontmatter.load(skill_path)
        metadata = post.metadata
        body = post.content

        # Validate frontmatter with Pydantic
        try:
            SkillFrontmatter(**metadata)
        except ValidationError as e:
            issues.extend(pydantic_errors_to_issues(e.errors(), section='frontmatter'))

        # Validate structure with Pydantic
        try:
            skill_struct = SkillStructure(body=body)

            # Add warnings if requested
            if show_warnings:
                for warning_msg in skill_struct.get_warnings():
                    issues.append(
                        skill_validators.ValidationIssue(
                            level=skill_validators.ValidationLevel.WARNING,
                            message=warning_msg,
                            section='structure',
                        )
                    )
        except ValidationError as e:
            issues.extend(pydantic_errors_to_issues(e.errors(), section='structure'))

        # Run custom validators that can't be replaced by Pydantic
        with open(skill_path, 'r', encoding='utf-8') as f:
            content = f.read()

        custom_validators = [
            UniquenessValidator(show_warnings, all_skills),
            CrossReferenceValidator(show_warnings, repo_root),
        ]

        for validator in custom_validators:
            validator_issues = validator.validate(skill_path, content, metadata, body)
            issues.extend(validator_issues)

    except (OSError, UnicodeDecodeError, KeyError) as e:
        issues.append(
            skill_validators.ValidationIssue(
                level=skill_validators.ValidationLevel.ERROR, message=f'Error validating skill: {e}'
            )
        )

    return skill_validators.ValidationResult(skill_path, issues)


def main():
    parser = argparse.ArgumentParser(
        description='Validate Agent Skills for GitHub Copilot',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Validate all skills
  ./scripts/validate-skills.py

  # Validate specific skill
  ./scripts/validate-skills.py .github/skills/my-skill/SKILL.md

  # CI mode (exit 1 on errors)
  ./scripts/validate-skills.py --ci

  # Show recommendations (warnings)
  ./scripts/validate-skills.py --recommend
        """,
    )

    parser.add_argument(
        'path',
        nargs='?',
        default='.',
        help='Path to skill file or directory to validate (default: current directory)',
    )
    parser.add_argument(
        '--ci', action='store_true', help='CI mode: exit with code 1 if any errors found'
    )
    parser.add_argument(
        '--recommend',
        action='store_true',
        help='Show recommendations and warnings in addition to errors',
    )

    args = parser.parse_args()

    # Determine repository root
    repo_root = os.path.abspath(os.getcwd())
    if os.path.exists(os.path.join(repo_root, '.git')):
        pass  # Already at repo root
    else:
        # Try to find repo root
        current = repo_root
        while current != '/':
            if os.path.exists(os.path.join(current, '.git')):
                repo_root = current
                break
            current = os.path.dirname(current)

    # Find skill files
    search_path = os.path.abspath(args.path)
    skill_files = find_skill_files(search_path)

    if not skill_files:
        print(f'No SKILL.md files found in {search_path}')
        return 0

    print(f'Validating {len(skill_files)} skill(s)...\n')

    # Load all skills for cross-validation
    all_skills = load_all_skills(skill_files)

    # Validate each skill
    results = []
    for skill_path in skill_files:
        try:
            result = validate_skill(skill_path, args.recommend, all_skills, repo_root)
            results.append(result)

        except (OSError, RuntimeError, ValueError) as e:
            print(f'✗ {skill_path}')
            print(f'  ✗ Error: {e}')
            results.append(skill_validators.ValidationResult(skill_path, []))

    # Display results
    passed_count = 0
    failed_count = 0
    warning_count = 0

    for result in results:
        print(result)

        if result.issues:
            for issue in result.issues:
                print(issue)
            print()

        if result.is_valid:
            passed_count += 1
            if result.has_warnings:
                warning_count += 1
        else:
            failed_count += 1

    # Summary
    print(f'\nSummary: {passed_count} passed, {failed_count} failed', end='')
    if warning_count > 0:
        print(f', {warning_count} with warnings', end='')
    print()

    # Exit code
    if args.ci and failed_count > 0:
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
