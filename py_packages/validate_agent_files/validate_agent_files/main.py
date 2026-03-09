#!/usr/bin/env python3

"""Main orchestration function for validate_agent_files."""

from __future__ import annotations

import sys
from typing import List, Optional

from .cli import parse_arguments
from .core import CustomizationsValidationEngine
from .formatters import format_results


def main(args: Optional[List[str]] = None) -> int:
    """Run the validate_agent_files tool."""
    parsed_args = parse_arguments(args)

    # Determine whether warnings should be shown based on CLI flags.
    # Start from the existing "recommend" behavior and allow explicit flags
    # like --no-warnings / --errors-only to disable warnings.
    show_warnings = getattr(parsed_args, 'recommend', True)
    if getattr(parsed_args, 'no_warnings', False) or getattr(parsed_args, 'errors_only', False):
        show_warnings = False

    engine = CustomizationsValidationEngine(show_warnings=show_warnings)
    results = engine.validate(parsed_args.path, parsed_args.kind)

    # Compute exit code before formatting so it can be reused for CI behavior.
    exit_code = 1 if any(not result.is_valid for result in results) else 0

    formatted = format_results(results, output_format=parsed_args.format)

    # Control output based on verbosity / CI / quiet flags. Use getattr with
    # defaults so this remains robust even if some flags are not defined.
    is_quiet = getattr(parsed_args, 'quiet', False)
    is_ci = getattr(parsed_args, 'ci', False)
    is_verbose = getattr(parsed_args, 'verbose', False)

    if not is_quiet and formatted:
        if not is_ci:
            # Default behavior (non-CI): always print formatted results.
            print(formatted)
        else:
            # CI mode: only print on failure, or when explicitly verbose.
            if exit_code != 0 or is_verbose:
                print(formatted)

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
