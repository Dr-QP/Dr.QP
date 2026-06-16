#!/usr/bin/env python3
"""Install generated overlay Python requirements in one pip invocation."""

from __future__ import annotations

import os
import re
import subprocess
import sys
import tempfile
from collections.abc import Iterator
from pathlib import Path


SKIP_REQUIREMENT_RE = re.compile(r"^\s*(#|\[|$)")


def iter_requires_files(search_roots: list[Path]) -> Iterator[Path]:
    for root in search_roots:
        if not root.is_dir():
            continue
        yield from root.rglob("requires.txt")


def collect_requirements(requires_files: list[Path]) -> tuple[list[str], list[Path]]:
    requirements: list[str] = []
    contributing_files: list[Path] = []

    for path in requires_files:
        path_requirements: list[str] = []
        with path.open("r", encoding="utf-8") as requires_file:
            for line in requires_file:
                if SKIP_REQUIREMENT_RE.match(line):
                    continue
                path_requirements.append(line.rstrip("\n"))

        if path_requirements:
            contributing_files.append(path)
            requirements.extend(path_requirements)

    # Deduplicate while preserving first-seen order, since the same package
    # can be listed in multiple requires.txt files.
    return list(dict.fromkeys(requirements)), contributing_files


def write_requirements_file(requirements: list[str]) -> Path:
    """
    Write requirements to a temp file so pip reads them with `-r`.

    Passing requirements as positional CLI arguments only avoids ARG_MAX up
    to ~2 MB; a temp file keeps this safe regardless of requirement count.
    """
    fd, path = tempfile.mkstemp(prefix="filtered-requires-", suffix=".txt")
    with os.fdopen(fd, "w", encoding="utf-8") as requirements_file:
        requirements_file.write("\n".join(requirements) + "\n")
    return Path(path)


def build_pip_command(constraints_file: Path, requirements_file: Path) -> list[str]:
    command = [
        "/usr/bin/python3",
        "-m",
        "pip",
        "install",
        "--break-system-packages",
        "--ignore-installed",
    ]

    if constraints_file.is_file():
        command.extend(["-c", str(constraints_file)])

    command.extend(["-r", str(requirements_file)])

    if os.geteuid() != 0:
        command.insert(0, "sudo")

    return command


def main() -> int:
    search_roots = [Path(arg) for arg in sys.argv[1:]]
    if not search_roots:
        search_roots = [Path("build"), Path("install")]

    requires_files = sorted(iter_requires_files(search_roots))
    requirements, contributing_files = collect_requirements(requires_files)
    if not requirements:
        return 0

    script_dir = Path(__file__).resolve().parent
    constraints_file = script_dir / "pip-constraints.txt"

    print("Installing requirements from:")
    for path in contributing_files:
        print(f"  {path}")

    requirements_file = write_requirements_file(requirements)
    try:
        subprocess.run(
            build_pip_command(constraints_file, requirements_file),
            check=True,
        )
    finally:
        requirements_file.unlink(missing_ok=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
