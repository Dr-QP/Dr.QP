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

"""CLI argument parsing for the validate-skills tool."""

import argparse
from typing import List, Optional


def parse_arguments(args: Optional[List[str]] = None) -> argparse.Namespace:
    """
    Parse command-line arguments.

    Args:
        args: List of argument strings (if None, sys.argv is used)

    Returns:
        Parsed arguments as Namespace

    """
    parser = argparse.ArgumentParser(
        description='Validate agent skills for compliance with best practices'
    )

    parser.add_argument(
        'path',
        nargs='?',
        default='.',
        help='Path to skill directory, skill file, or current directory (default: .)',
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
        choices=['text', 'json', 'csv', 'xml'],
        default='text',
        help='Output format (text, json, csv, xml)',
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
