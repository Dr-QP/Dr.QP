#!/usr/bin/env python3

"""CLI argument parsing for customization validation."""

from __future__ import annotations

import argparse
from typing import List, Optional


def parse_arguments(args: Optional[List[str]] = None) -> argparse.Namespace:
    """Parse command-line arguments for customization validation."""
    parser = argparse.ArgumentParser(
        description='Validate repository customizations for skills, agents, and prompts'
    )

    parser.add_argument(
        'path',
        nargs='?',
        default='.',
        help='Path to the customization root, file, or current directory (default: .)',
    )
    parser.add_argument(
        '--kind',
        choices=['all', 'skills', 'agents', 'prompts'],
        default='all',
        help='Customization kind to validate (default: all)',
    )
    parser.add_argument(
        '--recommend',
        action='store_true',
        default=False,
        help='Show recommendations for improvement',
    )
    parser.add_argument(
        '--ci',
        action='store_true',
        default=False,
        help='Enable CI mode (no colors, structured output)',
    )
    parser.add_argument(
        '--no-warnings',
        action='store_false',
        dest='warnings',
        default=True,
        help='Exclude warnings from validation results',
    )
    parser.add_argument(
        '--errors-only',
        action='store_true',
        default=False,
        help='Show only errors, exclude warnings',
    )
    parser.add_argument(
        '--format',
        choices=['text', 'json', 'csv'],
        default='text',
        help='Output format (text, json, csv)',
    )
    parser.add_argument(
        '-q', '--quiet', action='store_true', default=False, help='Suppress non-error output'
    )
    parser.add_argument(
        '-v', '--verbose', action='store_true', default=False, help='Show detailed output'
    )
    parser.add_argument(
        '--json',
        action='store_const',
        dest='format',
        const='json',
        help='Output results in JSON format (shorthand for --format json)',
    )
    parser.add_argument(
        '--csv',
        action='store_const',
        dest='format',
        const='csv',
        help='Output results in CSV format (shorthand for --format csv)',
    )

    return parser.parse_args(args)
