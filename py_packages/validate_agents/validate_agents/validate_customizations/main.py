#!/usr/bin/env python3

"""Main orchestration function for validate_customizations."""

from __future__ import annotations

import sys
from typing import List, Optional

from ..validate_skills.formatters import format_results
from .cli import parse_arguments
from .core import CustomizationsValidationEngine


def main(args: Optional[List[str]] = None) -> int:
    """Run the validate_customizations tool."""
    parsed_args = parse_arguments(args)
    engine = CustomizationsValidationEngine(show_warnings=parsed_args.recommend)
    results = engine.validate(parsed_args.path, parsed_args.kind)

    formatted = format_results(results, output_format=parsed_args.format)
    if formatted:
        print(formatted)

    return 1 if any(not result.is_valid for result in results) else 0


if __name__ == '__main__':
    sys.exit(main())
